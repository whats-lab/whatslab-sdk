# Quest 앱 (PoseDataTracker)

Meta Quest 에서 손/컨트롤러 포즈를 OSC 로 송신하는 앱.
`whatslab.receiver.quest.QuestHandReceiver`(핸드트래킹)와
`QuestControllerReceiver`(컨트롤러)가 이 앱의 패킷을 수신한다.

- `PoseDataTracker 1.0.5.apk` — 내장 배포본 (adb 설치용)

## 설치
```bash
../../scripts/install_quest_app.sh          # 이 apk 를 자동 탐색해 adb 설치
```
Quest 를 USB 연결하고 개발자 모드 + USB 디버깅을 켠 뒤, 헤드셋에서 '허용'을 수락한다.

## 버전 업데이트
새 apk 를 이 폴더에 넣고 **이전 버전은 지운다** — 스크립트가 `PoseDataTracker*.apk`
중 첫 번째(알파벳순)를 고르므로 두 개가 있으면 낮은 버전이 설치된다.
(`*.apk` 는 .gitignore 대상이지만 `assets/quest/*.apk` 는 예외로 커밋됨)
