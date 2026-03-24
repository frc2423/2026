import React, { useState } from "react";
import {
  useNTValue,
  useNTConnection,
} from "./store/useNetworktables";
import {
  BooleanBox,
  Canvas,
  CanvasMjpgStream,
  Gyro
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

  const [activeAlliance] = useNTValue<string>("/Robot/robotContainer/dashboardlogger/activeAlliance", "Active Alliance: [No NTValue Loaded]");
  const [currentShift] = useNTValue<string>("/Robot/robotContainer/dashboardlogger/currentShift", "Active Alliance: [No NTValue Loaded]");
  const [isAllianceActive] = useNTValue<boolean>("/Robot/robotContainer/dashboardlogger/isAllianceActive", true);
  const [isAllianceActiveNextShift] = useNTValue<boolean>("/Robot/robotContainer/dashboardlogger/isAllianceActiveNextShift", true);
  const [matchTime] = useNTValue<Number>("/Robot/robotContainer/dashboardlogger/matchTime", 160.0);
  const [shiftTimeRemaining] = useNTValue<Number>("/Robot/robotContainer/dashboardlogger/shiftTimeRemaining", 0.0);
  

const headerStyle: React.CSSProperties = {
  backgroundColor: "grey",
  color: 'white',
  fontWeight: '700',
  textAlign: 'center'
};

const divStyle: React.CSSProperties = {
  borderStyle: "solid",
  borderWidth: "4px",
  borderColor: "black",
  fontSize: '30px',
  width:"150px"
}

  return (
    <div style={{ padding: "20px" }}>
      {/* <Gyro  value={50}/> */}
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
      <div>
        <BooleanBox
          label={isRedAlliance ? "Red Alliance" : "Blue Alliance"}
          value={isRedAlliance}
          trueColor="red"
          falseColor="blue"
        />
      </div>
      {/* Element Container */}
      <div style={{ display: 'flex', gap: '10px'}}>
      {/* Active Alliance Content */}
      <div style ={divStyle}>
        <header style={{backgroundColor: activeAlliance === "both" ? "grey" : activeAlliance === "red" ? "red" : "blue",
  color: 'white',
  fontSize: '30px',
  fontWeight: '700', textAlign: 'center'}}>Active Alliance</header>
        {activeAlliance}
      </div>
      {/* Current Shift Content */}
      <div style ={divStyle}>
        <header style={headerStyle}>Current Shift</header>
        {currentShift}
      </div>
      {/* Active? Content */}
      <div style ={divStyle}>
        <header style={headerStyle}>Active?</header>
        <BooleanBox
          label={isAllianceActive ? "Yes" : "No"}
          value={isAllianceActive}
          trueColor="green"
          falseColor="red"
        />
      </div>
      </div>
      <br></br>
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