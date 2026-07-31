# Robot Show Orchestrator YAML 語法參考

本文件描述 Robot Show Orchestrator 第一版實際支援的 YAML 格式。YAML
只描述表演語意，不直接填寫 ROS topic、UART 封包、`show_run_id` 或
`command_id`。

## 1. 基本規則

- 使用空白縮排，建議每層 2 個空白，不可使用 Tab。
- `#` 後方是註解。
- 每個 workflow node 的 `id` 都可以省略，也可以重複。
- 若有填 `id`，它只作為錯誤訊息中的人類可讀標籤；建議使用容易閱讀的
  英文 snake_case，例如 `rail1_down`。
- 時間單位為秒；Servo 的 `step_delay_ms` 單位為毫秒。
- node `type`、`target` 和 `command` 必須使用文件列出的拼字。
- 有空白的字串建議加引號，例如 `face_type: "LOOK LEFT"`。

最小結構：

```yaml
schema_version: "1.0"
show:
  id: my_show
  defaults:
    on_failure: abort_show
    command_timeout_s: 30.0
  root:
    type: delay
    duration_s: 1.0
```

## 2. 最上層欄位

```yaml
schema_version: "1.0"
show:
  id: robot_table_demo
  name: 機械手臂表演桌台示範
  version: 1
  requirements:
    rail1_initial_state: TOP
    rail2_initial_state: TOP
  defaults:
    on_failure: abort_show
    command_timeout_s: 30.0
  root:
    # 一個 workflow node
```

欄位說明：

| 欄位 | 必要 | 說明 |
|---|---:|---|
| `schema_version` | 是 | 第一版固定為字串 `"1.0"`。 |
| `show.id` | 是 | 表演定義的 ID，不是單次執行 ID。 |
| `show.name` | 否 | 顯示名稱。 |
| `show.version` | 否 | 作者自行管理的版本。 |
| `show.requirements` | Rail 使用時必要 | 宣告表演開始前可信的 Rail 位置。 |
| `show.defaults.on_failure` | 否 | 第一版只支援 `abort_show`。 |
| `show.defaults.command_timeout_s` | 否 | Action 預設逾時，省略時為 30 秒。 |
| `show.root` | 是 | 表演的根 workflow node。 |

`rail1_initial_state` 和 `rail2_initial_state` 只接受 `TOP` 或 `BOTTOM`。
它們是操作人員確認後的假設，不是 sensor 測量值。

## 3. Workflow node 共通規則

第一版共有四種 node：

- `action`
- `sequence`
- `parallel`
- `delay`

每個 node 只需要填 `type`；`id` 是選填的顯示標籤：

```yaml
type: action
```

Orchestrator 每次開始表演時會產生 `show_run_id`，每次執行 node 時會產生
內部唯一的 `node_id`，並在每次真正發出 Action 時產生新的
`command_id`。因此作者不用為重複動作手動發明不同名稱。

下面三個 node 都合法：

```yaml
- id: happy
  type: action
  target: screen
  command: face
  args: {face_type: HAPPY}
- id: happy                 # id 重複也可以
  type: action
  target: screen
  command: face
  args: {face_type: HAPPY}
- type: action              # id 完全省略也可以
  target: screen
  command: face
  args: {face_type: HAPPY}
```

若 Action 失敗，狀態訊息會包含實際 YAML 檔案和行號，例如：

```text
/home/sean/shows/first_show.yaml:27: node "arm_move1" (arm joint_move)
FAILED: execution failed (MoveIt code -4)
```

省略 `id` 時會以動作內容顯示，例如 `node "arm joint_move"`。同名 node
仍可由不同的 YAML 行號辨認。

## 4. Action

Action 向一個裝置發出命令，等待該命令的正式完成結果。

```yaml
- id: servo1_open
  type: action
  target: servo1
  command: move_to
  args:
    target_degree: 30
    step_delay_ms: 10
  timeout_s: 5.0
```

共通欄位：

| 欄位 | 必要 | 說明 |
|---|---:|---|
| `id` | 否 | 人類可讀標籤；可以重複。省略時由動作內容代替。 |
| `type` | 是 | 固定為 `action`。 |
| `target` | 是 | 裝置名稱。 |
| `command` | 是 | 該裝置支援的語意命令。 |
| `args` | 是 | 命令參數。 |
| `timeout_s` | 否 | 正數；省略時使用 `defaults.command_timeout_s`。 |

### 4.1 Arm

```yaml
- id: arm_pos1
  type: action
  target: arm
  command: joint_move
  args:
    positions: [0.0, 0.0, 0.0, 0.0, 0.0, -0.3]
    speed_scaling: 0.5
  timeout_s: 20.0
```

限制：

- `positions` 必須剛好有 6 個有限數值，順序為
  `base, part1, part2, part3, part4, part5`。
- 單位為 radian。
- `speed_scaling` 必須在 `0.2～1.0`。
- 只有 MoveIt trajectory execution 回覆成功，Action 才算成功。
- Planning succeeded 不代表 Action 已完成。

YAML 中的 Arm 位置必須另外經過碰撞與實機安全確認。

### 4.2 Rail

```yaml
- id: rail1_down
  type: action
  target: rail1
  command: move
  args:
    steps: 16000
    direction: low
  timeout_s: 10.0
```

`target` 可為 `rail1` 或 `rail2`。第一版採二位置模型：

| 目前狀態 | 合法 direction | 完成後狀態 |
|---|---|---|
| `TOP` | `low` | `BOTTOM` |
| `BOTTOM` | `high` | `TOP` |

限制：

- `steps` 必須是整數 `16000`。
- `direction` 只能是 `low` 或 `high`。
- 使用 Rail 時必須在 `requirements` 宣告可信的初始位置。
- 不可從 `TOP` 執行 `high`，也不可從 `BOTTOM` 執行 `low`。
- ESP32 完成指定 step 數後才算成功。

Rail 沒有位置 sensor 或 encoder，因此成功代表 step pulse 程序完成，不代表
機構位置已被感測器驗證。取消、逾時或 UART 失敗後會標記為 `UNKNOWN`。

### 4.3 Servo

```yaml
- id: servo2_open
  type: action
  target: servo2
  command: move_to
  args:
    target_degree: 150
    step_delay_ms: 10
  timeout_s: 5.0
```

`target` 可為 `servo1` 或 `servo2`。

限制：

- `target_degree` 必須是 `0～180` 的整數。
- `step_delay_ms` 必須是正整數；ESP32 接受範圍為 `1～10000`。
- `step_delay_ms` 越大，移動越慢。
- 收到該 `command_id` 的 ESP32 完成回覆後才算成功。

Servo 沒有角度回授時，成功只代表控制程序完成。

### 4.4 Screen／FACE

```yaml
- id: screen_happy
  type: action
  target: screen
  command: face
  args:
    face_type: HAPPY
  timeout_s: 2.0
```

支援的 `face_type`：

```text
DEFAULT
HAPPY
ANGRY
TIRED
LOOK LEFT
LOOK RIGHT
LOOK UP
LOOK DOWN
LOOK CENTER
BLINK
WINK LEFT
WINK RIGHT
LAUGH
CONFUSED
IDLE ON
IDLE OFF
```

ESP32 處理 FACE 命令並回覆成功後，Action 就完成。若要讓表情維持一段時間，
應在下一個節點放 `delay`，不可把 FACE 的完成時間當成顯示時間。

## 5. Sequence

Sequence 依照 `children` 順序執行。前一個 child 成功後才會啟動下一個。

```yaml
- id: screen_sequence
  type: sequence
  children:
    - id: show_happy
      type: action
      target: screen
      command: face
      args:
        face_type: HAPPY
    - id: hold_happy
      type: delay
      duration_s: 1.5
    - id: show_default
      type: action
      target: screen
      command: face
      args:
        face_type: DEFAULT
```

`children` 必須是非空 list。

## 6. Parallel

Parallel 同時啟動所有 `branches`。第一版固定等待所有分支完成：

```yaml
- id: move_rails_together
  type: parallel
  join: all
  branches:
    - id: rail1_down
      type: action
      target: rail1
      command: move
      args: {steps: 16000, direction: low}
    - id: rail2_down
      type: action
      target: rail2
      command: move
      args: {steps: 16000, direction: low}
```

限制：

- `branches` 必須是非空 list。
- `join` 第一版只能是 `all`，省略時也視為 `all`。
- 不同 branch 不可使用相同裝置。

不合法範例：

```yaml
# 兩個平行 branch 都控制 arm，因此啟動前會被拒絕。
type: parallel
join: all
branches:
  - id: arm_a
    type: action
    target: arm
    command: joint_move
    args: {positions: [0, 0, 0, 0, 0, 0], speed_scaling: 0.5}
  - id: arm_b
    type: action
    target: arm
    command: joint_move
    args: {positions: [0, 0, 0, 0, 0, 0], speed_scaling: 0.5}
```

相同裝置在同一個 Sequence 中依序使用是合法的。不同裝置，例如 Arm、
Screen、rail1、rail2、servo1 和 servo2，可以平行。

如果任一平行分支失敗，`abort_show` 會取消其他仍在執行的 Action，並停止啟動
新的 node。

## 7. Delay

```yaml
- id: hold_expression
  type: delay
  duration_s: 1.5
```

- `duration_s` 必須是大於 0 的有限數值。
- Delay 只控制表演節奏，不可用來猜測馬達或 Arm 是否完成。
- Delay 開始後遇到 soft pause 仍會完成；下一個尚未開始的 node 會等 Resume。
- Stop 可以中斷 Delay。

## 8. Pause、Resume、Stop 與失敗

Soft Pause：

- 不啟動新的 node。
- 已開始的 Action 或 Delay 繼續。
- 完成結果會正常保存。
- Resume 後才啟動下一個 node。

Stop：

- 不再啟動新 node。
- 對執行中的 Arm 發出精確 cancel。
- 對執行中的 Rail／Servo 發出含相同 ID 的 ESP32 cancel。
- 無法確認位置的裝置標記為 `UNKNOWN`。

Action 的失敗可能包含參數拒絕、裝置忙碌、MoveIt planning／execution
失敗、UART 中斷、ESP32 未回覆、逾時或取消。第一版一律使用
`on_failure: abort_show`。

## 9. 啟動前驗證

Orchestrator 載入或開始表演時會檢查：

- YAML 結構和 schema version。
- node 的 `id` 若有填寫，必須是非空字串；可以重複。
- Action target、command 與參數範圍。
- Rail 初始狀態和 TOP／BOTTOM 轉移是否合法。
- Parallel branch 是否控制相同裝置。
- Commander 是否 ready。
- UART bridge 是否持續收到 ESP32 `PONG`。
- `safety_confirmed` 是否已由操作人員設為 true。

`safety_confirmed` 只能代表操作人員已做現場確認，不能取代實體急停、limit
switch、護欄或其他硬體安全設備。

## 10. 完整範例

可執行的完整格式範例位於：

```text
robot_show_orchestrator/config/demo_show.yaml
```

範例內的 Arm joint positions 只是格式示範。正式表演前必須換成已確認安全、
可達且無碰撞的實際位置。

## 11. 常見錯誤

```yaml
# 合法：ID 可以重複，也可以不填
- {id: action1, type: delay, duration_s: 1.0}
- {id: action1, type: delay, duration_s: 1.0}
- {type: delay, duration_s: 1.0}
```

```yaml
# 錯誤：Arm 少於六個 positions
positions: [0.0, 0.0, 0.0]
```

```yaml
# 錯誤：Servo 使用小數角度或 delay
target_degree: 90.5
step_delay_ms: 10.5
```

```yaml
# 錯誤：Rail 從 TOP 執行 high
requirements:
  rail1_initial_state: TOP
args:
  steps: 16000
  direction: high
```

```yaml
# 格式合法但概念錯誤：把 Delay 當成 Arm 完成判斷
- id: send_arm
  type: action
  target: arm
  command: joint_move
  args: {positions: [0, 0, 0, 0, 0, 0], speed_scaling: 0.5}
- id: assume_arm_done
  type: delay
  duration_s: 2.0
```

上例中的 Action 本身已經會等待 MoveIt execution result；不需要也不應用固定
Delay 判斷 Arm 是否完成。如果這 2 秒是刻意安排的表演節奏，才應保留該
Delay。
