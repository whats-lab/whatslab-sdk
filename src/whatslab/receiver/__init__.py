"""범용 입력 레이어.

Quest / AirGlove 등에서 InputSample 을 산출한다. 텔레옵과 무관하게 위치추적
용도로도 사용 가능하며, teleop 을 절대 import 하지 않는다.
python-osc 는 각 구현의 start() 에서 lazy import (모듈 import 만으로 강제 안 됨).

side 규약: 모든 리시버의 left/right 는 **물리적 기기의 좌/우**다 (채널 재해석 금지).
"왼손 컨트롤러를 오른손 글러브에 마운트" 같은 크로스핸드 조합은 **Model 계층**
(TeleopModel/GloveModel 의 arm_side/hand_side)에서 선언한다.

    from whatslab.receiver.quest_controller import QuestControllerReceiver
    from whatslab.receiver.quest_hand import QuestHandReceiver
    from whatslab.receiver.glove_human_hand import GloveHumanHandReceiver
    from whatslab.receiver.glove_robot_hand import GloveRobotHandReceiver
"""
from .glove_human_hand import GloveHumanHandReceiver
from .glove_robot_hand import GloveRobotHandReceiver
from .quest_controller import QuestControllerReceiver
from .quest_hand import QuestHandReceiver

__all__ = ["QuestControllerReceiver", "QuestHandReceiver",
           "GloveHumanHandReceiver", "GloveRobotHandReceiver"]
