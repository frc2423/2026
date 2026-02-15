import { useState } from "react";
import {
  useNTValue,
  useNTConnection,
  useRobotConnection,
} from "./nt3/useNetworktables";
import {
  BooleanBox,
  Canvas,
  CanvasMjpgStream,
} from "@frc-web-components/react";

function App() {
  // Monitor connection status
  const isConnected = useNTConnection();
  const isRobotConnected = useRobotConnection();

  // Get/Set a number value
  const [someNumber, setSomeNumber] = useNTValue<number>(
    "/SmartDashboard/someNumber",
    0
  );

  const [isRedAlliance] = useNTValue<boolean>("/FMSInfo/IsRedAlliance", false);
  const [streams] = useNTValue<string[]>(
    "/CameraPublisher/USB Camera 0/streams",
    []
  );
  const [quality, setQuality] = useState(50);
  const [fps, setFps] = useState(60);

  return (
    <div>
      <div>WebSocket Connected: {isConnected ? "Yes" : "No"}</div>
      <div>Robot Connected: {isRobotConnected ? "Yes" : "No"}</div>
      <div>Alliance: {isRedAlliance ? "Red" : "Blue"}</div>
      <div>
        <BooleanBox
          label="Alliance"
          value={isRedAlliance}
          trueColor="red"
          falseColor="blue"
        />
      </div>
      <div>
        Some number: {someNumber}
        <button onClick={() => setSomeNumber((someNumber ?? 0) + 1)}>
          Increment
        </button>
        <button onClick={() => setSomeNumber((someNumber ?? 0) - 1)}>
          Decrement
        </button>
        <button onClick={() => setSomeNumber(0)}>Reset</button>
      </div>
      <div>
        <Canvas>
          <CanvasMjpgStream
            srcs={streams}
            resolutionWidth={320}
            resolutionHeight={160}
            fps={fps}
            quality={quality}
          />
        </Canvas>
      </div>
      <div>
        <span>
          Quality: {quality}
          <input
            type="range"
            min="0"
            max="100"
            value={quality}
            onChange={(e) => setQuality(parseInt(e.target.value))}
          />
        </span>
        <span>
          FPS: {fps}
          <input
            type="range"
            min="1"
            max="60"
            value={fps}
            onChange={(e) => setFps(parseInt(e.target.value))}
          />
        </span>
      </div>
    </div>
  );
}

export default App;