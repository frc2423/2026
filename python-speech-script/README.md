# Robot Speech

Speaks text published to the NetworkTables `/robotSpeech` topic out loud, using
text-to-speech. Useful for getting audible feedback from an FRC robot (or the
WPILib simulator) on a connected laptop.

## How it works

`robot_speech.py` starts a NetworkTables 4 client, subscribes to a string topic
(default `/robotSpeech`), and speaks any text that gets published to it via the
[`pyttsx3`](https://pyttsx3.readthedocs.io/) engine in `speech.py`.

## Requirements

- [uv](https://docs.astral.sh/uv/) for dependency management
- Python 3.13+ (uv can install this for you)

Dependencies are declared in `pyproject.toml` and pinned in `uv.lock`:

- `pyntcore` — provides the `ntcore` NetworkTables module
- `pyttsx3` — cross-platform text-to-speech engine

## Install

Clone the repo, then sync the environment from the lockfile:

```bash
uv sync
```

This creates a `.venv` and installs the exact locked dependency versions.

## Run

Use `uv run` so the script executes inside the project environment.

Connect to a robot by team number:

```bash
uv run python robot_speech.py --team 1234
```

Connect to a specific server host/IP (overrides `--team`):

```bash
uv run python robot_speech.py --server 10.12.34.2
```

Connect to the WPILib simulator running on your machine (the default if neither
`--team` nor `--server` is given):

```bash
uv run python robot_speech.py --server localhost
```

The script runs until you stop it with `Ctrl+C`.

### Options

| Flag       | Description                                              | Default         |
| ---------- | -------------------------------------------------------- | --------------- |
| `--team`   | FRC team number to connect to.                           | _none_          |
| `--server` | Server host/IP to connect to (overrides `--team`).       | _none_          |
| `--topic`  | NetworkTables topic to listen on.                        | `/robotSpeech`  |

If neither `--team` nor `--server` is supplied, the client connects to
`localhost`.

## Publishing speech

Have your robot code (or any NetworkTables client) publish a string to the
`/robotSpeech` topic. Each new value is spoken aloud. For example, from another
Python NetworkTables client:

```python
import ntcore

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("speech_publisher")
inst.setServer("localhost")

pub = inst.getStringTopic("/robotSpeech").publish()
pub.set("Robot enabled")
```
