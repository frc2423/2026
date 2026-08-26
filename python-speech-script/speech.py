import pyttsx3
import threading
import time


class Speaker:
    def __init__(self, rate=180, volume=1.0, voice=None):
        self.rate = rate                 # Words per minute
        self.volume = volume             # 0.0 - 1.0
        self.voice = voice               # voice id, or name substring; None = default
        self._engine = None
        self._thread = None
        self._stop = threading.Event()

    def _make_engine(self):
        engine = pyttsx3.init()
        engine.setProperty("rate", self.rate)
        engine.setProperty("volume", self.volume)
        if self.voice:
            voice_id = self._resolve_voice(engine, self.voice)
            if voice_id is not None:
                engine.setProperty("voice", voice_id)
        return engine

    @staticmethod
    def _resolve_voice(engine, voice):
        """Return a voice id matching an exact id or a name substring."""
        voices = engine.getProperty("voices")
        for v in voices:
            if v.id == voice:
                return v.id
        lowered = voice.lower()
        for v in voices:
            if lowered in v.name.lower():
                return v.id
        return None

    @staticmethod
    def list_voices():
        """Return a list of (name, id) tuples for the installed voices."""
        engine = pyttsx3.init()
        try:
            return [(v.name, v.id) for v in engine.getProperty("voices")]
        finally:
            engine.stop()

    def say(self, text):
        """Speak text and block until finished."""
        self._engine = self._make_engine()
        self._engine.say(text)
        self._engine.runAndWait()
        self._engine = None

    def say_async(self, text):
        """Speak text on a background thread; returns immediately."""
        self._stop.clear()
        self._thread = threading.Thread(target=self.say, args=(text,))
        self._thread.start()
        return self._thread

    def countdown(self, start):
        """Count down from start to 1, speaking each number."""
        for number in range(start, 0, -1):
            if self._stop.is_set():
                break
            self.say(str(number))

    def countdown_async(self, start):
        """Run countdown on a background thread; returns immediately."""
        self._stop.clear()
        self._thread = threading.Thread(target=self.countdown, args=(start,))
        self._thread.start()
        return self._thread

    def stop(self, wait=True):
        """Interrupt current speech and cancel any pending sequence."""
        self._stop.set()
        if self._engine is not None:
            self._engine.stop()
        if wait:
            self.wait()

    def wait(self):
        """Block until any background speech finishes."""
        if self._thread is not None:
            self._thread.join()


if __name__ == "__main__":
    for name, voice_id in Speaker.list_voices():
        print(f"{name} -> {voice_id}")

    # A slower rate with a deeper voice sounds more mechanical/robotic.
    speaker = Speaker(rate=130, voice="David")
    speaker.say("Robot enabled.")
    speaker.countdown_async(5)
    speaker.wait()
    time.sleep(3)
    speaker.stop()
    speaker.say("Countdown stopped.")
    speaker.countdown_async(5)
    time.sleep(3)
    speaker.stop()
    speaker.say("Countdown stopped.")
