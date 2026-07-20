# Quest 앱 (PoseDataTracker)

Meta Quest 에서 손/컨트롤러 포즈를 OSC 로 송신하는 앱. `whatslab.receiver.meta.QuestReceiver`
가 이 앱의 패킷을 수신한다.

- `PoseDataTracker-1.0.0.apk` — 내장 배포본 (adb 설치용)

## 설치
```bash
../../scripts/install_quest_app.sh          # 이 apk 를 자동 탐색해 adb 설치
```
Quest 를 USB 연결하고 개발자 모드 + USB 디버깅을 켠 뒤, 헤드셋에서 '허용'을 수락한다.

## 버전 업데이트
새 apk 를 이 폴더에 `PoseDataTracker-<버전>.apk` 로 넣으면 스크립트가 자동으로 찾는다.
(`*.apk` 는 .gitignore 대상이지만 `assets/quest/*.apk` 는 예외로 커밋됨)
