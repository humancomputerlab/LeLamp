"""Build a single-file, offline, bilingual (中文 / English) 3D assembly animation from the official LeLamp URDF.

Usage: python3 build_assembly_animation.py [--out FILE]
Needs: Python 3.9+, numpy. three.js is downloaded once from the npm registry into .cache/.
Output: LeLamp_Assembly_Animation.html (open it in any modern browser; no network needed).
"""
from __future__ import annotations

import argparse
import base64
import gzip
import hashlib
import io
import json
import tarfile
import urllib.request
from pathlib import Path

import numpy as np

from lelamp_urdf import load_visuals

HERE = Path(__file__).resolve().parent
OUT = HERE / "LeLamp_Assembly_Animation.html"
THREE_VERSION = "0.149.0"
THREE_URL = f"https://registry.npmjs.org/three/-/three-{THREE_VERSION}.tgz"
THREE_SHA256 = "ab8b7c131f6ce72e05081dff4882046584e4a4ce5ad864f9f2a0e3146cc3997c"

# Wrist roll turned so wrist_pitch's geared horn is on the same side as the other pitch servos
# (official rule), wrist pitch turned so the head looks forward-down.
POSE = {"4": 2.4958, "5": 1.05}

SERVO_PARTS = ("motor_1723_3", "xg_ziji_16", "sg_ziji_15", "zk_122", "ge_27", "pcb_chazuo_92")

SERVOS = {
    1: ("scs215_v5", "base_yaw", "底座偏航", "base yaw"),
    2: ("lamparm__wrist_head", "base_pitch", "底座俯仰", "base pitch"),
    3: ("lamparm__base_elbow", "elbow_pitch", "肘部俯仰", "elbow pitch"),
    4: ("lamparm__elbow_wrist", "wrist_roll", "腕部翻滚", "wrist roll"),
    5: ("lamparm__wrist_head_2", "wrist_pitch", "腕部俯仰", "wrist pitch"),
}
# Direction each servo body slides into its pocket (checked against the pocket geometry).
SERVO_ENTRY = {1: (0, 0, 1), 2: (0, 0, 1), 3: (0.3, 0, 0.95), 4: (0.25, -0.97, 0), 5: (0.97, -0.25, 0)}
PITCH = {2, 3, 5}
# Centre of the cable hole in the base floor, relative to the yaw servo, after align_base() (measured from lamp_base.stl).
WIRE_HOLE_FROM_YAW = (-32.0, 0.0)


def T(zh: str, en: str) -> dict:
    return {"zh": zh, "en": en}


def unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def align_base(visuals) -> None:
    """Turn the base link about the yaw axis so its long side lies along X, as in the official top-down
    reference (docs/assets/images/body/3_body.jpg): arm along the base, cable hole on the -X side, front = -Y."""
    base = [v for v in visuals if v.link == "scs215_v5"]
    xy = np.concatenate([v.tris.reshape(-1, 3) for v in base if v.mesh == "lamp_base.stl"])[:, :2]
    horn = np.concatenate([v.tris.reshape(-1, 3) for v in base if v.part.startswith("金属舵盘_驱动")])
    pivot = horn.mean(0)[:2]

    def rotation(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s], [s, c]])

    def extents(a):
        return np.ptp((xy - pivot) @ rotation(a).T, axis=0)

    angles = np.radians(np.arange(-45, 45, 0.05))
    a = angles[int(np.argmin([np.prod(extents(a)) for a in angles]))]
    ex, ey = extents(a)
    if ex < ey:
        a += np.pi / 2
    for v in base:
        pts = v.tris.reshape(-1, 3)
        pts[:, :2] = (pts[:, :2] - pivot) @ rotation(a).T + pivot


def build_items(visuals):
    def pick(link, cond):
        tris = [v.tris for v in visuals if v.link == link and cond(v.part)]
        if not tris:
            raise ValueError(f"no visual matched in {link}")
        return np.concatenate(tris)

    items: dict[str, dict] = {}

    def add(item_id, kind, label, tris):
        pts = tris.reshape(-1, 3)
        items[item_id] = {"id": item_id, "kind": kind, "label": label, "tris": tris, "centroid": pts.mean(0)}

    add("base", "print", T("底座 LampBase", "LampBase"), pick("scs215_v5", lambda p: p == "lamp_base"))
    add("cover", "cover", T("底盖 LampBase - Cover", "LampBase - Cover"), pick("scs215_v5", lambda p: p == "lamp_base_cover"))
    add("cup1", "print", T("LampArm (Pitch) #1 小方杯", "LampArm (Pitch) #1"),
        pick("lamparm__wrist_head", lambda p: p == "lamparm__wrist_head"))
    add("arm1", "print", T("下段臂 LampArm (Base-Elbow)", "LampArm (Base-Elbow)"),
        pick("lamparm__base_elbow", lambda p: p == "lamparm__base_elbow"))
    add("arm2", "print", T("上段臂 LampArm (Elbow-Wrist)", "LampArm (Elbow-Wrist)"),
        pick("lamparm__elbow_wrist", lambda p: p == "lamparm__elbow_wrist"))
    add("cup2", "print", T("LampArm (Pitch) #2 小方杯", "LampArm (Pitch) #2"),
        pick("lamparm__wrist_head_2", lambda p: p == "lamparm__wrist_head_2"))
    add("head", "print", T("灯头 LampHead", "LampHead"), pick("diffuser", lambda p: p == "lamphead"))
    add("diffuser", "diffuser", T("扩散罩 LampHead - Diffuser", "LampHead - Diffuser"), pick("diffuser", lambda p: p == "diffuser"))

    for n, (link, name, zh, en) in SERVOS.items():
        add(f"s{n}", "servo", T(f"{n} 号 {zh} {name}", f"ID {n} {en} ({name})"), pick(link, lambda p: p.startswith(SERVO_PARTS)))
        add(f"h{n}d", "horn_drive", T("带齿舵盘（铜齿那面，拧螺丝）", "Geared horn (copper-gear side, screwed)"),
            pick(link, lambda p: p.startswith("金属舵盘_驱动")))
        if n in PITCH:
            add(f"h{n}i", "horn_idle", T("无齿舵盘（背面，只套上不拧）", "Plain horn (back side, not screwed)"),
                pick(link, lambda p: p.startswith("金属舵盘_从动")))
    return items


def horn_axis(items, horn_id, servo_id) -> np.ndarray:
    pts = items[horn_id]["tris"].reshape(-1, 3)
    c = pts.mean(0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    n = vt[2]
    if n @ (c - items[servo_id]["centroid"]) < 0:
        n = -n
    return n


def build_steps(items):
    steps = []
    c = lambda i: items[i]["centroid"]
    front_text = T("带齿舵盘朝这边（灯的前方）", "Geared horn faces here (lamp front)")

    def servo_step(n, title, text, warn, dist):
        sid, entry = f"s{n}", unit(SERVO_ENTRY[n]) * 120
        horns = [f"h{n}d"] + ([f"h{n}i"] if n in PITCH else [])
        anim = [{"id": sid, "keys": [[0, entry], [0.45, entry], [1, [0, 0, 0]]]}]
        for h in horns:
            out = entry + horn_axis(items, h, sid) * 45
            anim.append({"id": h, "keys": [[0, out], [0.4, entry], [0.45, entry], [1, [0, 0, 0]]]})
        step = {"title": title, "text": text, "warn": warn, "anim": anim, "dist": dist,
                "focus": c(sid) + entry * 0.45,
                "labels": [{"text": items[sid]["label"], "at": c(sid), "big": True, "follow": sid}]}
        if n in PITCH:
            step["arrows"] = [{"from": c(f"h{n}d"), "dir": horn_axis(items, f"h{n}d", sid), "len": 75,
                               "text": front_text, "follow": f"h{n}d"}]
        else:
            step["labels"].append({"text": T("只装带齿舵盘", "Geared horn only"), "at": c(f"h{n}d"), "big": False, "follow": f"h{n}d"})
        steps.append(step)

    def part_step(item_id, direction, title, text, warn, dist, extra=(), label_at=None):
        off = unit(direction) * 110
        anim = [{"id": item_id, "keys": [[0, off], [1, [0, 0, 0]]]}]
        anim += [{"id": e, "keys": [[0, off], [1, [0, 0, 0]]]} for e in extra]
        steps.append({"title": title, "text": text, "warn": warn, "anim": anim, "dist": dist,
                      "focus": c(item_id) + off * 0.3,
                      "labels": [{"text": items[item_id]["label"], "at": c(item_id) if label_at is None else label_at,
                                  "big": True, "follow": item_id}]})
        return steps[-1]

    steps.append({
        "title": T("总览：五颗舵机分别装在哪", "Overview: where each servo goes"), "preview": True, "spin": True,
        "text": T("这是装好后的样子。开始前，先按官方文档《2. Servos Setup》运行 lelamp.setup_motors，给五颗舵机写好 ID 1–5 并贴上标签，"
                  "装的时候按标签对号入座。",
                  "This is the finished lamp. Before you start, follow docs “2. Servos Setup”: run lelamp.setup_motors to give "
                  "the five servos IDs 1–5 and label them, then put each servo where its label says."),
        "warn": T("全程断电装配：舵机电源和树莓派电源都不要接。", "Assemble with everything unpowered: no servo supply, no Raspberry Pi power."),
        "anim": [], "dist": 950,
        # Servos 4 and 5 sit close together, so servo 4's label goes underneath.
        "labels": [{"text": T(f"{n} 号 {zh}", f"ID {n} {name}"), "at": c(f"s{n}"), "big": True, "lift": -45 if n == 4 else 42}
                   for n, (_, name, zh, _) in SERVOS.items()]})

    hole = np.array([*(c("s1")[:2] + WIRE_HOLE_FROM_YAW), 40.0])
    b = items["base"]["tris"].reshape(-1, 3)
    front_from = np.array([c("base")[0], b[:, 1].min() - 5, 20.0])
    s = part_step("base", (0, 0, 1), T("第 1 步：放好底座，认清“前方”", "Step 1: Place the base and find the front"),
                  T("底座 LampBase 正面朝上放在桌上（装底盖的开口朝下，底盖最后再装）。官方约定：从上往下看，让出线孔（旧版是出线槽）在你右手边，"
                    "远离你的那一侧就是灯的“前方”。后面 2、3、5 号俯仰舵机的带齿舵盘都要朝前方。",
                    "Put LampBase on the table top side up (the open side for the cover faces down; the cover goes on last). "
                    "Official convention: looking down with "
                    "the cable hole (a slit on older bases) on your right, the side facing away from you is the lamp's front. "
                    "The geared horns of pitch servos 2, 3 and 5 all face this front."),
                  T("开始前：舵机电源和树莓派电源全部断开。", "Before you start: servo supply and Raspberry Pi power disconnected."), 650,
                  label_at=c("base") + (35, 45, 0))
    # Official top-down reference view: camera behind the lamp (+Y), cable hole on the viewer's right.
    s["view"] = [np.pi, 0.95]
    s["labels"].append({"text": T("出线孔", "Cable hole"), "at": hole, "big": False, "follow": "base"})
    s["arrows"] = [{"from": front_from, "dir": (0, -1, 0), "len": 70, "text": T("前方", "Front"), "follow": "base"}]

    servo_step(1, T("第 2 步：1 号舵机（底座偏航）", "Step 2: Servo 1 (base yaw)"),
               T("先在桌上把带齿舵盘压到舵机铜齿那面，拧好中心螺丝；再把舵机舵盘朝上放进底座中间的舵机仓，用舵机附带的螺丝固定。",
                 "On the table, press the geared horn onto the copper-gear side and tighten its centre screw. Then drop the "
                 "servo, horn up, into the pocket in the middle of the base and fix it with the screws that came with the servo."),
               T("偏航舵机只装带齿舵盘，背面不装无齿舵盘。压舵盘时不要用力扭舵机轴。",
                 "The yaw servo gets the geared horn only; no plain horn on the back. Don't force the shaft round while pressing the horn on."),
               540)
    part_step("cup1", (0, 0, 1), T("第 3 步：LampArm (Pitch) #1", "Step 3: LampArm (Pitch) #1"),
              T("小方杯开口朝上，杯底的孔对准 1 号舵机的舵盘，用螺丝拧紧。",
                "Cup opening up. Line its bottom holes up with servo 1's horn and screw it on."),
              T("杯子的转角决定灯臂朝哪。建议和动画一致：之后的灯臂沿底座长边、朝远离出线孔的方向伸出，再拧螺丝。",
                "The cup's rotation decides where the arm points. Match the animation: the arm should run along the long "
                "side of the base, away from the cable hole, before you tighten the screws."), 400)
    servo_step(2, T("第 4 步：2 号舵机（底座俯仰）", "Step 4: Servo 2 (base pitch)"),
               T("两面都装舵盘：带齿舵盘装在铜齿面（拧螺丝），无齿舵盘装在背面（只套上，不拧，起对轴作用）。然后把舵机竖着插进小方杯。",
                 "Horns on both sides: the geared horn on the copper-gear side (screwed), the plain horn on the back (just "
                 "slipped on, it only keeps the axis aligned). Then slide the servo upright into the cup."),
               T("金色箭头那一侧就是带齿舵盘，必须朝灯的前方。2、3、5 号三颗俯仰舵机都朝同一侧，装反了灯臂会反向运动。",
                 "The gold arrow marks the geared horn; it must face the lamp front. Pitch servos 2, 3 and 5 all face the "
                 "same way — if one is flipped, that joint moves backwards."), 420)
    part_step("arm1", (0, 0, 1), T("第 5 步：下段臂 LampArm (Base-Elbow)", "Step 5: LampArm (Base-Elbow)"),
              T("下段臂底部的 U 形叉从上往下套，两边分别夹住 2 号舵机的带齿舵盘和无齿舵盘，两边都拧螺丝。",
                "Lower the U-fork at the bottom of this arm over servo 2 so it grips the geared horn on one side and the "
                "plain horn on the other. Screw both sides."),
              T("先对孔再拧。孔位对不上时，把舵盘拔下来转一个齿再压上，不要硬扭舵机轴。",
                "Line up the holes before screwing. If they don't line up, pull the horn off and re-seat it one tooth "
                "over instead of forcing the shaft."), 750)
    servo_step(3, T("第 6 步：3 号舵机（肘部俯仰）", "Step 6: Servo 3 (elbow pitch)"),
               T("同样两面装舵盘，然后从斜上方放进下段臂顶端的舵机座。",
                 "Horns on both sides again, then lower the servo at an angle into the seat at the top of the lower arm."),
               T("带齿舵盘和 2 号同一侧，朝灯的前方（金色箭头）。", "Geared horn on the same side as servo 2, facing the lamp front (gold arrow)."), 420)
    part_step("arm2", (1, 0, 0.3), T("第 7 步：上段臂 LampArm (Elbow-Wrist)", "Step 7: LampArm (Elbow-Wrist)"),
              T("上段臂一端的 U 形叉夹住 3 号舵机两侧的舵盘，两边拧螺丝；另一端是方形舵机盒，留给 4 号舵机。",
                "The U-fork at one end grips both horns of servo 3; screw both sides. The square box at the other end is for servo 4."),
              T("U 形叉那端接肘部，方盒那端朝灯头，不要装反。", "Fork end at the elbow, box end towards the head — don't swap them."), 700)
    servo_step(4, T("第 8 步：4 号舵机（腕部翻滚）", "Step 8: Servo 4 (wrist roll)"),
               T("只装带齿舵盘。从侧面开口把舵机放进上段臂末端的方盒，舵盘顺着灯臂方向朝外伸出。",
                 "Geared horn only. Slide the servo into the box at the end of the upper arm through its side opening, "
                 "with the horn pointing outwards along the arm."),
               T("翻滚舵机和偏航舵机一样，只装带齿舵盘。", "Like the yaw servo, the roll servo gets the geared horn only."), 420)
    part_step("cup2", (1, 0, 0), T("第 9 步：LampArm (Pitch) #2", "Step 9: LampArm (Pitch) #2"),
              T("第二个小方杯的杯底孔对准 4 号舵盘拧紧，杯口朝外（朝灯头方向）。",
                "Screw the second cup onto servo 4's horn, opening facing outwards (towards the head)."),
              T("这一步决定灯头会不会歪：要让接下来 5 号舵机的带齿舵盘和 2、3 号在同一侧。",
                "This sets whether the head sits level: servo 5's geared horn must end up on the same side as servos 2 and 3."), 400)
    servo_step(5, T("第 10 步：5 号舵机（腕部俯仰）", "Step 10: Servo 5 (wrist pitch)"),
               T("两面装舵盘，然后插进第二个小方杯。", "Horns on both sides, then slide it into the second cup."),
               T("带齿舵盘仍然和 2、3 号同一侧，朝灯的前方（金色箭头）。", "Geared horn on the same side as servos 2 and 3, facing the lamp front (gold arrow)."), 420)
    part_step("head", unit(c("head") - c("s5")), T("第 11 步：灯头 LampHead", "Step 11: LampHead"),
              T("先按官方文档《3. LeLamp Assembly》第 1 节，把喇叭和 LED 矩阵装进灯头、焊好连到底座的线（官方用拆开的网线，至少 50 cm）。"
                "然后用灯头颈部的 U 形叉夹住 5 号舵机两侧的舵盘，两边拧螺丝，最后装上扩散罩。",
                "First follow section 1 of docs “3. LeLamp Assembly”: fit the speaker and LED matrix in the head and solder "
                "the cable that runs to the base (a stripped Ethernet cable, at least 50 cm). Then let the head's U-fork grip "
                "both horns of servo 5, screw both sides, and fit the diffuser."),
              T("灯头线一路沿灯臂留到底座，经过每个关节都留出弯曲余量。", "Run the head cable down the arm to the base, leaving slack at every joint."),
              520, extra=("diffuser",))
    steps.append({"title": T("第 12 步：舵机串联接线", "Step 12: Daisy-chain the servos"), "wiring": True,
                  "text": T("所有舵机都装好后再接线。官方顺序：从 5 号腕部俯仰开始，5 → 4 → 3 → 2 → 1，1 号再接驱动板。"
                            "每颗舵机上的两个接口是并联的，不分进出。线沿着灯臂用扎带固定。",
                            "Wire up only after every servo is mounted. Official order: start at servo 5 (wrist pitch), "
                            "5 → 4 → 3 → 2 → 1, then servo 1 to the driver board. The two ports on each servo are wired in "
                            "parallel, so either one works. Tie the cables along the arm."),
                  "warn": T("每个关节都要留弯曲余量：关节转到两端时，线不能被拉紧或夹住。原装线不够长就按官方文档接长。",
                            "Leave slack at every joint: at both ends of travel no cable may be pulled tight or pinched. "
                            "If a stock cable is too short, extend it as the official guide describes."),
                  "anim": [], "dist": 950,
                  "labels": [{"text": T(f"{n} 号", f"ID {n}"), "at": c(f"s{n}"), "big": True} for n in SERVOS]})
    part_step("cover", (0, 0, -1), T("第 13 步：底座电子件，盖底盖", "Step 13: Base electronics, then the cover"),
              T("把驱动板和树莓派固定在底座里，两者用 USB-C 线相连；装上 ReSpeaker 扩展板，按原理图把灯头线接到扩展板。"
                "全部检查无误后再盖上底盖。",
                "Mount the servo driver and the Raspberry Pi in the base and connect them with a USB-C cable. Fit the "
                "ReSpeaker HAT and connect the head cable to it as in the schematic. Check everything, then close the cover."),
              T("通电前再查一遍接线。舵机和树莓派各用一个电源（官方：舵机 5V 2A 直流，树莓派 5V 2A Type-C）；"
                "舵机电压必须和你买的 STS3215 版本匹配。",
                "Re-check the wiring before powering on. Servos and Pi get separate supplies (official guide: 5 V 2 A DC for "
                "the servos, 5 V 2 A USB-C for the Pi); the servo voltage must match your STS3215 version."), 700)
    steps.append({"title": T("完成！", "Done!"), "spin": True, "anim": [], "dist": 950, "labels": [],
                  "text": T("装配完成，拖动鼠标从各个角度检查。下一步：按《4. LeLamp Setup》测试舵机，再按《5. LeLamp Control》标定。",
                            "Assembly done — drag to check it from every side. Next: test the servos with docs “4. LeLamp Setup”, "
                            "then calibrate with “5. LeLamp Control”."),
                  "warn": T("标定时托住灯臂；1 号偏航和 4 号腕部翻滚只各转约 ±90°，不要整圈转。",
                            "Support the arm during calibration. Turn servo 1 (base yaw) and servo 4 (wrist roll) only about "
                            "±90° — never a full turn.")})
    return steps


def pack(items, order):
    allpts = np.concatenate([items[i]["tris"].reshape(-1, 3) for i in order])
    lo, hi = allpts.min(0) - 1, allpts.max(0) + 1
    blobs, meta = [], []
    offset = 0
    for i in order:
        pts = items[i]["tris"].reshape(-1, 3)
        q = np.round((pts - lo) / (hi - lo) * 65535).astype(np.uint16)
        uniq, inv = np.unique(q, axis=0, return_inverse=True)
        idx = inv.reshape(-1).astype(np.uint32)
        pos_bytes, idx_bytes = uniq.tobytes(), idx.tobytes()
        meta.append({"id": i, "kind": items[i]["kind"],
                     "pos": [offset, len(uniq)], "idx": [offset + len(pos_bytes) + (-len(pos_bytes) % 4), len(idx)]})
        pad = b"\0" * (-len(pos_bytes) % 4)
        blobs += [pos_bytes, pad, idx_bytes]
        offset += len(pos_bytes) + len(pad) + len(idx_bytes)
    data = gzip.compress(b"".join(blobs), compresslevel=9, mtime=0)
    return data, meta, lo.tolist(), hi.tolist()


def to_json(o):
    if isinstance(o, np.ndarray):
        return [round(float(x), 2) for x in o]
    if isinstance(o, (np.floating, np.integer)):
        return round(float(o), 2)
    raise TypeError(type(o))


def three_js() -> str:
    path = HERE / ".cache" / f"three-{THREE_VERSION}.min.js"
    if not path.exists():
        with urllib.request.urlopen(THREE_URL, timeout=60) as r:
            tgz = r.read()
        digest = hashlib.sha256(tgz).hexdigest()
        if digest != THREE_SHA256:
            raise RuntimeError(f"{THREE_URL}: sha256 {digest} does not match {THREE_SHA256}")
        with tarfile.open(fileobj=io.BytesIO(tgz), mode="r:gz") as tar:
            js = tar.extractfile("package/build/three.min.js").read()
        path.parent.mkdir(exist_ok=True)
        path.write_bytes(js)
    return path.read_text(encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()

    visuals = load_visuals(POSE)
    align_base(visuals)
    items = build_items(visuals)
    steps = build_steps(items)
    order = list(items)
    data, meta, lo, hi = pack(items, order)
    servos = [{"n": n, "at": items[f"s{n}"]["centroid"]} for n in SERVOS]
    base_c = items["base"]["centroid"]
    driver = [base_c[0] + 45, base_c[1] - 30, 25.0]
    payload = {"items": meta, "lo": lo, "hi": hi, "steps": steps, "servos": servos, "driver": driver,
               "centroids": {i: items[i]["centroid"] for i in order}}
    template = (HERE / "template.html").read_text(encoding="utf-8")
    html = (template.replace("/*__THREE__*/", three_js())
            .replace("__META__", json.dumps(payload, ensure_ascii=False, default=to_json))
            .replace("__DATA__", base64.b64encode(data).decode()))
    args.out.write_text(html, encoding="utf-8")
    tri_count = sum(len(items[i]["tris"]) for i in order)
    print(f"wrote {args.out.name}: {args.out.stat().st_size / 1e6:.1f} MB, {len(order)} items, {tri_count} triangles, {len(steps)} steps")


if __name__ == "__main__":
    main()
