# CLAUDE.md

이 저장소의 규칙은 [CONTRIBUTING.md](CONTRIBUTING.md)(한국어판 [CONTRIBUTING.ko.md](CONTRIBUTING.ko.md)) 다. 작업 전에 읽고 그대로 따른다.
사실(설치, 지원 손, 공개 API)은 [README](README.ko.md) 와 [API.md](docs/API.md) 가 기준이다 — 여기에 옮겨 적지 않는다.

Claude 에게만 해당하는 것:

- 코드에 docstring·주석을 쓰지 않는다. 설명하고 싶으면 대화나 문서에 쓴다.
- 커밋·PR 에 `Co-Authored-By: Claude`, `Generated with Claude Code` 같은 표기를 넣지 않는다.
- SDK 를 쓰는 다른 저장소에 맞추려고 이 저장소에 호환 코드를 넣지 않는다. 필요하면 그쪽을 고치자고 제안한다.
- 범위(손 전용)를 넓히는 변경은 하기 전에 묻는다.
- 공개 이름·시그니처를 바꾸면 `git grep` 으로 옛 이름이 코드·문서·설정 어디에도 남지 않았는지 확인한다.
- 끝났다고 말하기 전에 `ruff check .` 와 `pytest -q` 를 돌리고 결과를 그대로 보고한다.
