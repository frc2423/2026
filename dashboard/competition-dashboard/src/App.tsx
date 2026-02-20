import { useState } from "react";
import {
  useNTValue,
  useNTConnection,
} from "./store/useNetworktables";
import {
  BooleanBox,
  Canvas,
  CanvasMjpgStream,
} from "@frc-web-components/react";

function App() {
  // Monitor connection status
  const { isConnected, address, connect } = useNTConnection();

  // Local state for address input
  const [addressInput, setAddressInput] = useState(address);

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
    <div style={{ padding: "20px" }}>
      {/* NT4 Connection Panel */}
      <div
        style={{
          display: "flex",
          gap: "15px",
          alignItems: "center",
          marginBottom: "20px",
          padding: "10px",
          backgroundColor: "#f5f5f5",
          borderRadius: "4px",
        }}
      >
        {/* Connection Status */}
        <div>
          <span
            style={{
              color: isConnected ? "green" : "red",
              fontWeight: "bold",
              fontSize: "18px",
            }}
          >
            {isConnected ? "●" : "○"}
          </span>
        </div>

        {/* Address Input */}
        <div style={{ flex: 1 }}>
          <input
            type="text"
            value={addressInput}
            onChange={(e) => {
              setAddressInput(e.target.value);
              connect(e.target.value);
            }}
            placeholder="NT4 Address (localhost, 10.TE.AM.2, or team number)"
            style={{
              width: "100%",
              padding: "8px",
              fontSize: "14px",
              border: "1px solid #ccc",
              borderRadius: "4px",
            }}
          />
        </div>

        {/* Current Address Display */}
        <div style={{ fontSize: "12px", color: "#666" }}>
          Connected to: <span style={{ fontFamily: "monospace" }}>{address}</span>
        </div>
      </div>

      {/* Dashboard Content */}
      <div>WebSocket Connected: {isConnected ? "Yes" : "No"}</div>
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