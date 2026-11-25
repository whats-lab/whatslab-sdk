const SDK = "https://github.com/whats-lab/whatslab-sdk";
const HANDS = "https://github.com/whats-lab/dexterous-hand-urdf";
const WEBXR = "https://github.com/immersive-web/webxr-input-profiles";
const link = (href, text) => `<a href="${href}" rel="noopener">${text}</a>`;

const STRINGS = {
  en: {
    "title": "whatslab-sdk hand preview",
    "sub": "whatslab-sdk · Dexterous hand preview",
    "menu": "Menu",
    "nav.intro": "Intro",
    "nav.hands": "Hands",
    "nav.about": "About",
    "search": "Search name, maker, license",
    "intro.title": "Dexterous hands you can teleoperate",
    "intro.p1": `This site is built with ${link(SDK, "whatslab-sdk")} by WHATs LAB Corp. The hands are collected in ${link(HANDS, "dexterous-hand-urdf")} from each maker's official repository. Every hand shown here can be teleoperated with whatslab-sdk: DexPilot retargeting from OpenXR hand tracking or an AirGlove.`,
    "intro.p2": "Pick a hand under Hands in the sidebar to watch it follow a human hand through six synthetic motions, retargeted ahead of time with the DexPilot config bundled in the SDK.",
    "how.title": "How the retargeting works",
    "how.items": [
      "Input: the 26 OpenXR hand joints, from an AirGlove (through Spine) or Quest hand tracking.",
      "DexPilot: the robot's wrist-to-fingertip and fingertip-to-fingertip vectors are fitted to the scaled human vectors with a Huber loss. As the thumb nears a finger, that pair's target ramps continuously to a pinch distance.",
      "Posture prior: the human bend and spread angles give each robot joint a target q̄; spread targets fade as the knuckle (MCP) bends.",
      "SLSQP solves within the joint limits, at most 10 iterations per frame (about 0.7 ms per hand, median, DG-5F-S). A frame that cannot be solved keeps the previous command, and the output passes a first-order low-pass filter.",
    ],
    "use.title": "Use it",
    "use.install": "Install",
    "use.retarget": "Retarget one frame of OpenXR joints. xr_joints is a (26, 8) array (qx qy qz qw px py pz radius) from an AirGlove or Quest hand tracking.",
    "use.meshes": "Meshes are downloaded to a local cache on demand:",
    "copy": "Copy",
    "copied": "Copied",
    "about.title": "About",
    "about.items": [
      `URDFs come from each maker's or lab's official repository and are collected, with any fixes listed in each hand's <span class="mono">PATCHES.txt</span>, in ${link(HANDS, "dexterous-hand-urdf")}. Every hand page links its source repository, commit, license and license files.`,
      "Meshes are not rehosted. Your browser downloads the upstream originals at a pinned commit, so the first visit to a hand can take a while (a few MB to tens of MB, shown on each hand page); after that the browser cache makes it fast.",
      `The human hand is the WebXR generic hand from ${link(WEBXR, "immersive-web/webxr-input-profiles")} (MIT, Copyright 2019 Amazon).`,
      `Retargeting runs in ${link(SDK, "whatslab-sdk")} 1.0.0, which supports the 42 hands of dexhand-description 1.0.0 with a bundled DexPilot config for each.`,
    ],
    "speed": "Speed",
    "right": "Right",
    "left": "Left",
    "human.off": "Robot only",
    "human.beside": "Human beside",
    "human.overlay": "Human overlay",
    "motion.fist": "Flat to fist",
    "motion.pinch_index": "Pinch thumb–index",
    "motion.pinch_middle": "Pinch thumb–middle",
    "motion.pinch_ring": "Pinch thumb–ring",
    "motion.pinch_little": "Pinch thumb–little",
    "motion.v_pose": "V pose",
    "downloading": "Downloading meshes…",
    "loading.text": "{pct}% · {done}/{files} files · {mb} MB",
    "loading.failed": "{n} failed",
    "status.failed": "{n} of the model's mesh files could not be downloaded.",
    "status.model": "The model could not be loaded.",
    "meta": "{maker} · {dof} actuated DoF · {fingers} fingers · {license}",
    "stat.dof": "actuated DoF",
    "stat.fingers": "fingers",
    "stat.mesh": "mesh download, {n} files",
    "stat.nomesh": "meshes",
    "stat.notshown": "not shown",
    "joints": "Joints",
    "source": "Source",
    "maker": "Maker",
    "license": "License",
    "model": "Model",
    "files": "Files",
    "usehand": "Use this hand",
    "note.mirrored": "This repository has no {side} model of this hand. The {other} model is shown, driven by the mirrored motion.",
    "note.offline": "Joint angles are precomputed offline with whatslab-sdk DexPilot retargeting (config <span class=\"mono\">{config}</span>) from synthetic hand motions.",
    "note.noconfig": "whatslab-sdk has no retargeting config for this hand yet, so only the model is shown.",
    "note.unclear": "License unclear upstream. Included for non-commercial research use; see the folder's license notes.",
    "side.right": "right",
    "side.left": "left",
    "empty": "No hand matches “{q}”. Try a maker name or a license.",
  },
  ko: {
    "title": "whatslab-sdk 손 미리보기",
    "sub": "whatslab-sdk · 로봇 손 미리보기",
    "menu": "메뉴",
    "nav.intro": "소개",
    "nav.hands": "손 목록",
    "nav.about": "정보",
    "search": "이름, 제조사, 라이선스 검색",
    "intro.title": "원격 조종할 수 있는 로봇 손",
    "intro.p1": `이 사이트는 주식회사 왓츠랩(WHATs LAB Corp)의 ${link(SDK, "whatslab-sdk")}로 만들었습니다. 손은 각 제조사의 공식 저장소에서 가져와 ${link(HANDS, "dexterous-hand-urdf")}에 모았습니다. 여기 있는 모든 손은 whatslab-sdk 로 원격 조종할 수 있습니다. OpenXR 손 추적이나 AirGlove 입력을 DexPilot 으로 리타게팅합니다.`,
    "intro.p2": "사이드바의 손 목록에서 손을 고르면 사람 손을 따라 움직이는 모습을 볼 수 있습니다. 합성 동작 여섯 가지를 SDK 에 동봉된 DexPilot 설정으로 미리 리타게팅한 것입니다.",
    "how.title": "리타게팅 방법",
    "how.items": [
      "입력: OpenXR 손 관절 26개입니다. AirGlove(Spine 경유)나 Quest 손 추적에서 받습니다.",
      "DexPilot: 로봇의 손목→손끝 벡터와 손끝 사이 벡터를 배율을 맞춘 사람 벡터에 Huber 손실로 맞춥니다. 엄지가 다른 손가락에 가까워지면 그 쌍의 목표가 핀치 거리로 연속해서 바뀝니다.",
      "자세 사전항: 사람 손의 굽힘·벌림 각으로 로봇 관절마다 목표 q̄ 를 정합니다. 손가락 뿌리(MCP)가 굽을수록 벌림 목표는 줄어듭니다.",
      "관절 한계 안에서 SLSQP 로 풉니다. 프레임당 반복은 최대 10회이고 손 하나에 약 0.7 ms(중앙값, DG-5F-S)가 걸립니다. 풀지 못한 프레임은 직전 명령을 유지하고, 출력은 1차 저역통과 필터를 거칩니다.",
    ],
    "use.title": "사용 방법",
    "use.install": "설치",
    "use.retarget": "OpenXR 관절 한 프레임을 리타게팅합니다. xr_joints 는 AirGlove 나 Quest 손 추적에서 받은 (26, 8) 배열(qx qy qz qw px py pz radius)입니다.",
    "use.meshes": "메시는 필요할 때 로컬 캐시로 내려받습니다:",
    "copy": "복사",
    "copied": "복사됨",
    "about.title": "정보",
    "about.items": [
      `URDF 는 각 제조사나 연구실의 공식 저장소에서 가져와 ${link(HANDS, "dexterous-hand-urdf")}에 모았고, 고친 내용은 손마다 <span class="mono">PATCHES.txt</span> 에 적었습니다. 손 페이지마다 원본 저장소, 커밋, 라이선스와 라이선스 파일을 연결해 두었습니다.`,
      "메시는 다시 올리지 않습니다. 브라우저가 고정된 커밋의 업스트림 원본을 직접 받으므로 손을 처음 열 때는 시간이 걸릴 수 있습니다(손마다 수 MB~수십 MB, 손 페이지에 표시). 두 번째부터는 브라우저 캐시 덕분에 빠릅니다.",
      `사람 손은 ${link(WEBXR, "immersive-web/webxr-input-profiles")}의 WebXR generic hand 입니다(MIT, Copyright 2019 Amazon).`,
      `리타게팅은 ${link(SDK, "whatslab-sdk")} 1.0.0 에서 돌아갑니다. dexhand-description 1.0.0 의 손 42개를 지원하고 손마다 DexPilot 설정을 동봉합니다.`,
    ],
    "speed": "속도",
    "right": "오른손",
    "left": "왼손",
    "human.off": "로봇만",
    "human.beside": "사람 손 옆에",
    "human.overlay": "사람 손 겹쳐",
    "motion.fist": "펴기 → 주먹",
    "motion.pinch_index": "엄지–검지 집기",
    "motion.pinch_middle": "엄지–중지 집기",
    "motion.pinch_ring": "엄지–약지 집기",
    "motion.pinch_little": "엄지–소지 집기",
    "motion.v_pose": "V 자세",
    "downloading": "메시 다운로드 중…",
    "loading.text": "{pct}% · 파일 {done}/{files} · {mb} MB",
    "loading.failed": "{n}개 실패",
    "status.failed": "메시 파일 {n}개를 받지 못했습니다.",
    "status.model": "모델을 불러오지 못했습니다.",
    "meta": "{maker} · 구동 자유도 {dof} · 손가락 {fingers}개 · {license}",
    "stat.dof": "구동 자유도",
    "stat.fingers": "손가락",
    "stat.mesh": "메시 다운로드, 파일 {n}개",
    "stat.nomesh": "메시",
    "stat.notshown": "표시 안 함",
    "joints": "관절",
    "source": "출처",
    "maker": "제조사",
    "license": "라이선스",
    "model": "모델",
    "files": "파일",
    "usehand": "이 손 사용하기",
    "note.mirrored": "이 저장소에는 이 손의 {side} 모델이 없습니다. {other} 모델을 거울상 동작으로 보여 줍니다.",
    "note.offline": "관절각은 합성 손 동작을 whatslab-sdk DexPilot 리타게팅(설정 <span class=\"mono\">{config}</span>)으로 미리 계산한 값입니다.",
    "note.noconfig": "whatslab-sdk 에 이 손의 리타게팅 설정이 아직 없어 모델만 보여 줍니다.",
    "note.unclear": "업스트림 라이선스가 불분명합니다. 비상업 연구용으로 포함했으며, 폴더의 라이선스 안내를 확인해 주세요.",
    "side.right": "오른손",
    "side.left": "왼손",
    "empty": "“{q}”에 맞는 손이 없습니다. 제조사 이름이나 라이선스로 찾아보세요.",
  },
};

const store = navigator.cookieEnabled ? window.localStorage : null;
const fromUrl = new URLSearchParams(location.search).get("lang");
const saved = store ? store.getItem("lang") : null;
const browser = (navigator.language || "en").toLowerCase().startsWith("ko") ? "ko" : "en";
let lang = [fromUrl, saved, browser].find((l) => l === "en" || l === "ko");

export function currentLang() {
  return lang;
}

export function setLang(next) {
  lang = next;
  if (store) store.setItem("lang", next);
  const url = new URL(location.href);
  url.searchParams.set("lang", next);
  history.replaceState(history.state, "", url);
  document.documentElement.lang = next;
}

export function t(key, vars = {}) {
  const value = STRINGS[lang][key] ?? STRINGS.en[key] ?? key;
  if (Array.isArray(value)) return value;
  return value.replace(/\{(\w+)\}/g, (_, k) => (k in vars ? String(vars[k]) : `{${k}}`));
}
