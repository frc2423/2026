import argparse
import time

import ntcore

from speech import Speaker

TOPIC = "/robotSpeech"


def main():
    parser = argparse.ArgumentParser(
        description="Speak text published to a NetworkTables topic."
    )
    parser.add_argument(
        "--team", type=int, default=None, help="FRC team number to connect to."
    )
    parser.add_argument(
        "--server",
        type=str,
        default=None,
        help="Server host/IP to connect to (overrides --team).",
    )
    parser.add_argument(
        "--topic", default=TOPIC, help=f"Topic to listen to (default {TOPIC})."
    )
    args = parser.parse_args()

    speaker = Speaker()

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("robot_speech")

    if args.server:
        inst.setServer(args.server)
    elif args.team is not None:
        inst.setServerTeam(args.team)
        inst.startDSClient()
    else:
        inst.setServer("localhost")

    sub = inst.getStringTopic(args.topic).subscribe("")

    def on_change(event):
        text = event.data.value.getString()
        if text:
            print(f"Speaking: {text}")
            speaker.stop()
            speaker.say_async(text)

    inst.addListener(sub, ntcore.EventFlags.kValueAll, on_change)

    print(f"Listening for speech on {args.topic} (Ctrl+C to quit)...")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down.")
    finally:
        speaker.stop()
        inst.stopClient()


if __name__ == "__main__":
    main()