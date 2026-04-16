import React, { useState, useEffect, useRef, useMemo, type FormEvent } from "react";
import { useNTValue, useNTConnection } from "./store/useNetworktables";
import { Canvas, CanvasMjpgStream, Field, FieldRobot, FieldPath } from "@frc-web-components/react";
import {
  Box,
  Button,
  Card,
  Container,
  FormControl,
  Grid,
  InputLabel,
  MenuItem,
  Select,
  Slider,
  Stack,
  Tab,
  Tabs,
  TextField,
  Typography,
  ThemeProvider,
  createTheme,
  CssBaseline
} from "@mui/material";
import NumberField from "./components/NumberField";
import SendableChooser from "./components/SendableChooser";
import ShiftTimeline, { type Shift } from "./components/ShiftTimeline";

// 1. Bubbly, Vibrant Theme
// KWARQS 2423 — mallard teal-green & yellow
const KWARQS_GREEN = '#1d7c68';
const KWARQS_YELLOW = '#ffd426';

// 🦆 Checkerboard duck emoji background pattern
const DUCK_PATTERN = `url("data:image/svg+xml,${encodeURIComponent(
  `<svg xmlns='http://www.w3.org/2000/svg' width='100' height='100'>` +
  `<g opacity='0.25' font-size='28' font-family='Apple Color Emoji,Segoe UI Emoji,Noto Color Emoji,sans-serif'>` +
  `<text x='10' y='40'>🦆</text>` +
  `<text x='60' y='90'>🦆</text>` +
  `</g>` +
  `</svg>`
)}")`;


const bubblyTheme = createTheme({
  palette: {
    mode: 'dark',
    primary: { main: KWARQS_GREEN },
    background: { default: '#0a1a0d', paper: 'rgba(15, 35, 18, 0.6)' },
  },
  shape: { borderRadius: 32 },
  typography: {
    fontFamily: '"Nunito", "Inter", sans-serif',
    h4: { fontWeight: 800, letterSpacing: '-0.5px' },
    h6: { fontWeight: 700 },
    subtitle2: { textTransform: 'uppercase', letterSpacing: '1.5px', fontSize: '0.8rem', opacity: 0.9, fontWeight: 700 }
  },
  components: {
    MuiCssBaseline: {
      styleOverrides: {
        body: {
          background: `${DUCK_PATTERN} repeat, linear-gradient(135deg, #1a1a1a 0%, #212121 100%)`,
          backgroundAttachment: 'fixed',
          height: '100vh',
          overflow: 'hidden',
        },
        '*::-webkit-scrollbar': {
          width: '6px',
        },
        '*::-webkit-scrollbar-track': {
          background: 'transparent',
        },
        '*::-webkit-scrollbar-thumb': {
          background: 'rgba(255,255,255,0.2)',
          borderRadius: '3px',
        },
        '*::-webkit-scrollbar-thumb:hover': {
          background: 'rgba(255,255,255,0.35)',
        },
      }
    },
    MuiCard: {
      styleOverrides: {
        root: {
          backdropFilter: 'blur(16px)',
          border: '1px solid rgba(255,255,255,0.15)',
          boxShadow: '0 8px 32px rgba(0,0,0,0.4)',
        }
      }
    },
    MuiButton: {
      styleOverrides: {
        root: {
          borderRadius: '50px',
          fontWeight: 800,
          fontSize: '1.1rem',
          textTransform: 'none',
          padding: '14px 28px',
          boxShadow: '0 4px 15px rgba(0,0,0,0.2)'
        }
      }
    },
    MuiOutlinedInput: {
      styleOverrides: { root: { borderRadius: '20px', backgroundColor: 'rgba(0,0,0,0.2)' } }
    },
    MuiMenu: {
      styleOverrides: {
        paper: {
          backgroundColor: '#2a2a2a',
          backdropFilter: 'none',
          border: '1px solid rgba(255,255,255,0.15)',
          boxShadow: '0 8px 32px rgba(0,0,0,0.5)',
        }
      }
    }
  }
});

// Gradients — KWARQS green/yellow team colours
const gradients = {
  glass: `rgba(37, 150, 190, 0.18)`,
  red: 'linear-gradient(135deg, #ff0844 0%, #ffb199 100%)',
  blue: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)',
  green: `linear-gradient(135deg, ${KWARQS_GREEN} 0%, #56d97a 100%)`,
  warning: `linear-gradient(135deg, ${KWARQS_YELLOW} 0%, #ffb800 100%)`,
  kwarqs: `linear-gradient(135deg, ${KWARQS_GREEN} 0%, ${KWARQS_YELLOW} 100%)`,
};

// 2. Bubbly StatCard: Takes a gradient background for active states
const StatCard = ({ title, children, bgGradient = gradients.glass }: { title: string, children: React.ReactNode, bgGradient?: string }) => (
  <Card sx={{
    height: "100%", p: 1.5, display: "flex", flexDirection: "column", justifyContent: "space-between",
    background: bgGradient,
    transition: 'transform 0.2s ease-in-out',
    '&:hover': { transform: 'translateY(-2px)' }
  }}>
    <Typography variant="subtitle2" sx={{ mb: 1, textShadow: '0 2px 4px rgba(0,0,0,0.3)', fontSize: '0.7rem' }}>{title}</Typography>
    <Box sx={{ display: "flex", alignItems: "center", textShadow: '0 2px 10px rgba(0,0,0,0.4)' }}>
      {children}
    </Box>
  </Card>
);

// Alliance indicator — compact pill for top bar
const AllianceIndicator = ({ isRedAlliance }: { isRedAlliance: boolean }) => (
  <Card sx={{
    py: 0.75, px: 2,
    background: isRedAlliance ? gradients.red : gradients.blue,
    display: 'flex', alignItems: 'center', whiteSpace: 'nowrap', flexShrink: 0
  }}>
    <Typography sx={{ textTransform: 'uppercase', fontWeight: 800, fontSize: '0.75rem', textShadow: '0 2px 5px rgba(0,0,0,0.4)', letterSpacing: '1px' }}>
      {isRedAlliance ? "🔴 Red" : "🔵 Blue"}
    </Typography>
  </Card>
);

// ── Shift schedule (edit to match the actual game shift schedule) ─────────────
// Total match = 15 s auto + 135 s teleop = 150 s.
// Alliance values: 'red' | 'blue' | 'both'
const MATCH_DURATION = 160;

function getShifts(autoWinner: 'red' | 'blue' | 'both') {
  let autoLoser = 'both';
  if (autoWinner === 'red') {
    autoLoser = 'blue';
  } else if (autoWinner === 'blue') {
    autoLoser = 'red';
  }
  const SHIFTS: Shift[] = [
    { name: 'Auto', startSec: 0, endSec: 20, alliance: 'both' },
    { name: 'Transition', startSec: 20, endSec: 30, alliance: 'both' },
    { name: 'Shift 1', startSec: 30, endSec: 55, alliance: autoLoser },
    { name: 'Shift 2', startSec: 55, endSec: 80, alliance: autoWinner },
    { name: 'Shift 3', startSec: 80, endSec: 105, alliance: autoLoser },
    { name: 'Shift 4', startSec: 105, endSec: 130, alliance: autoWinner },
    { name: 'Endgame', startSec: 130, endSec: 160, alliance: 'both' },
  ];

  return SHIFTS;
}

function App() {
  const { isConnected, address, connect } = useNTConnection();
  const [addressInput, setAddressInput] = useState(address);
  const [activeTab, setActiveTab] = useState<"auto" | "teleop">("auto");

  const [, setSomeStringArray] = useNTValue<string[]>("/SmartDashboard/someStringArray");
  const [, setSomeNumberArray] = useNTValue<number[]>("/SmartDashboard/setSomeNumberArray");

  const handleSubmit = (e: FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    const formData = new FormData(e.currentTarget);
    const stepNames = formData.getAll("stepName") as string[];
    const stepDelays = formData.getAll("stepDelay").map((val) => parseFloat(val as string));
    setSomeStringArray(stepNames);
    setSomeNumberArray(stepDelays);
  };

  const [isRedAllianceRaw] = useNTValue<boolean>("/FMSInfo/IsRedAlliance", false);
  const isRedAlliance = isRedAllianceRaw ?? false;
  const [streams] = useNTValue<string[]>("/CameraPublisher/USB Camera 0/streams", []);
  const [quality, setQuality] = useState(50);
  const [fps, setFps] = useState(60);

  const [_activeAlliance] = useNTValue<string>("/Robot/robotContainer/dashboardlogger/activeAlliance", "No Data");
  const [currentShift] = useNTValue<string>("/Robot/robotContainer/dashboardlogger/currentShift", "No Data");
  const [isAllianceActive] = useNTValue<boolean>("/Robot/robotContainer/dashboardlogger/isAllianceActive", true);
  const [_isAllianceActiveNextShift] = useNTValue<boolean>("/Robot/robotContainer/dashboardlogger/isAllianceActiveNextShift", true);
  const [winningAutoAlliance] = useNTValue<boolean>("/Robot/robotContainer/dashboardlogger/winningAutoAlliance", 'red');
    
    
  const [camerasConnected] = useNTValue<boolean>("/visionDebug/april_tag_cam/camerasConnected", true);
  const [twindexerJammed] = useNTValue<boolean>("/Robot/robotContainer/twindexer/isJammed", false);
  const [robotPose] = useNTValue<number[]>("/Pose/Robot", [0, 0, 0]);
  const [matchTime] = useNTValue<number>("/Robot/robotContainer/dashboardlogger/matchTime", -1);


  const shifts = useMemo(() => {
    return getShifts(winningAutoAlliance);
  }, [winningAutoAlliance]);

  // matchTime = remaining seconds in the *current* phase (resets each phase).
  // Track phase transitions so we can add future-phase time during auto.
  const phaseTransitionedRef = useRef(false); // true once auto → teleop seen
  const telepDurationRef = useRef(135);       // updated when transition observed
  const phaseInitialRef = useRef(-1);
  const prevMatchTimeRef = useRef(-1);

  useEffect(() => {
    const mt = matchTime ?? -1;
    const prev = prevMatchTimeRef.current;
    if (mt < 0) {
      // Match not running — reset
      phaseTransitionedRef.current = false;
      phaseInitialRef.current = -1;
    } else if (phaseInitialRef.current < 0) {
      phaseInitialRef.current = mt;
    } else if (prev >= 0 && mt > prev + 10) {
      // matchTime jumped up: auto → teleop transition
      phaseTransitionedRef.current = true;
      telepDurationRef.current = mt;
      phaseInitialRef.current = mt;
    }
    prevMatchTimeRef.current = mt;
  }, [matchTime]);

  const mt = matchTime ?? -1;
  // In teleop if we saw a transition, or matchTime > 20s (heuristic for mid-match connect)
  const inTeleop = phaseTransitionedRef.current || (mt >= 0 && mt > 20);
  const totalRemaining = mt < 0 ? -1 : inTeleop ? mt : mt + telepDurationRef.current;

  const formatMatchTime = (seconds: number) => {
    if (seconds < 0) return "--:--";
    const m = Math.floor(seconds / 60);
    const s = Math.floor(seconds % 60);
    return `${m}:${String(s).padStart(2, "0")}`;
  };


  return (
    <ThemeProvider theme={bubblyTheme}>
      <CssBaseline />
      <Container maxWidth={false} sx={{ py: 1, px: { xs: 1, md: 2 } }}>

        {/* Top Navigation / Status Bar */}
        <Stack direction="row" spacing={1.5} sx={{ mb: 1.5, alignItems: 'center' }}>
          <Card sx={{ p: 0.75, px: 2, display: 'flex', alignItems: 'center', gap: 1.5, flex: 1, maxWidth: 420, background: gradients.glass }}>
            <Box sx={{
              width: 12, height: 12, borderRadius: '50%', flexShrink: 0,
              background: isConnected ? gradients.green : gradients.red,
              boxShadow: isConnected ? '0 0 10px #0ba360' : '0 0 10px #ff0844'
            }} />
            <TextField
              size="small"
              fullWidth
              placeholder="NT4 Address (10.TE.AM.2)"
              value={addressInput}
              onChange={(e) => {
                setAddressInput(e.target.value);
                connect(e.target.value);
              }}
              sx={{ '& .MuiOutlinedInput-notchedOutline': { border: 'none' }, '& .MuiInputBase-input': { py: 0.5, fontSize: '0.85rem' } }}
            />
          </Card>

          <AllianceIndicator isRedAlliance={isRedAlliance} />

          {/* Match Timer */}
          <Card sx={{ py: 0.5, px: 2, background: gradients.glass, display: 'flex', flexDirection: 'column', alignItems: 'center', flexShrink: 0 }}>
            <Typography sx={{ fontSize: '0.6rem', opacity: 0.5, textTransform: 'uppercase', letterSpacing: '1px', lineHeight: 1.2 }}>Match Time</Typography>
            <Typography sx={{
              fontSize: '1.1rem', fontWeight: 800, letterSpacing: '2px', lineHeight: 1.2,
              color: totalRemaining >= 0 && totalRemaining <= 15 ? KWARQS_YELLOW : 'inherit',
              textShadow: totalRemaining >= 0 && totalRemaining <= 15 ? `0 0 12px ${KWARQS_YELLOW}` : 'none',
            }}>
              {formatMatchTime(totalRemaining)}
            </Typography>
          </Card>

          {/* Tab Switcher */}
          <Card sx={{ p: 0.25, background: gradients.glass, ml: 'auto !important' }}>
            <Tabs
              value={activeTab}
              onChange={(_, val) => setActiveTab(val)}
              sx={{ minHeight: 36, '& .MuiTabs-indicator': { background: gradients.kwarqs, height: 3, borderRadius: 2 } }}
            >
              <Tab label="🤖 Auto" value="auto" sx={{ fontWeight: 800, fontSize: '0.85rem', textTransform: 'none', minHeight: 36, py: 0.5, px: 1.5 }} />
              <Tab label="🕹️ Teleop" value="teleop" sx={{ fontWeight: 800, fontSize: '0.85rem', textTransform: 'none', minHeight: 36, py: 0.5, px: 1.5 }} />
            </Tabs>
          </Card>
        </Stack>

        {/* ── AUTO TAB ── */}
        {activeTab === "auto" && (
          <Stack spacing={1.5}>

            <Grid container spacing={2}>
              {/* Auto Routine Form */}
              <Grid size={{ xs: 12, lg: 4 }}>
                <Card sx={{ p: 2, height: '100%', background: gradients.glass }}>
                  <Typography variant="h6" sx={{ mb: 2, fontSize: '0.95rem' }}>Autonomous Routine Setup</Typography>
                  <form onSubmit={handleSubmit}>
                    <Stack spacing={2}>

                      {/* Step 1 */}
                      <Stack direction="row" spacing={1.5}>
                        <FormControl fullWidth size="small">
                          <InputLabel>Step 1 Action</InputLabel>
                          <Select name="stepName" label="Step 1 Action" defaultValue="none" sx={{ borderRadius: '16px', backgroundColor: 'rgba(255,255,255,0.08)' }}>
                            <MenuItem value="shoot">Shoot</MenuItem>
                            <MenuItem value="brief">Brief Delay</MenuItem>
                            <MenuItem value="outpost or depot">Go to Outpost/Depot</MenuItem>
                            <MenuItem value="none">None</MenuItem>
                          </Select>
                        </FormControl>
                        <Box sx={{ width: 120 }}><NumberField name="stepDelay" label="Delay (s)" min={0} max={20} defaultValue={0} /></Box>
                      </Stack>

                      {/* Step 2 */}
                      <Stack direction="row" spacing={1.5}>
                        <FormControl fullWidth size="small">
                          <InputLabel>Step 2 Action</InputLabel>
                          <Select name="stepName" label="Step 2 Action" defaultValue="trench" sx={{ borderRadius: '16px', backgroundColor: 'rgba(255,255,255,0.08)' }}>
                            <MenuItem value="trench">Go Under Trench</MenuItem>
                            <MenuItem value="bump">Go Over Bump</MenuItem>
                          </Select>
                        </FormControl>
                        <Box sx={{ width: 120 }}><NumberField name="stepDelay" label="Delay (s)" min={0} max={20} defaultValue={14} /></Box>
                      </Stack>

                      {/* Step 3 */}
                      <Stack direction="row" spacing={1.5}>
                        <FormControl fullWidth size="small">
                          <InputLabel>Step 3 Action</InputLabel>
                          <Select name="stepName" label="Step 3 Action" defaultValue="bump" sx={{ borderRadius: '16px', backgroundColor: 'rgba(255,255,255,0.08)' }}>
                            <MenuItem value="trench">Go Under Trench</MenuItem>
                            <MenuItem value="bump">Go Over Bump</MenuItem>
                            <MenuItem value="outpost or depot">Go to Outpost/Depot</MenuItem>
                            <MenuItem value="shoot">Shoot</MenuItem>
                            <MenuItem value="collect">Collect Fuel</MenuItem>
                          </Select>
                        </FormControl>
                      </Stack>

                      <Box>
                        <Button type="submit" variant="contained" fullWidth size="small" sx={{
                          background: gradients.kwarqs,
                          color: '#071a09', fontSize: '0.85rem', py: 0.75,
                          '&:hover': { background: `linear-gradient(135deg, #25a342 0%, #d4aa00 100%)` }
                        }}>
                          Deploy Routine
                        </Button>
                      </Box>
                    </Stack>
                  </form>
                </Card>
              </Grid>

              {/* Auto Chooser */}
              <Grid size={{ xs: 12, lg: 2 }}>
                <Card sx={{ p: 2, height: '100%', background: gradients.glass }}>
                  <Typography variant="subtitle2" sx={{ mb: 1.5, fontSize: '0.75rem', opacity: 0.7 }}>Auto Chooser</Typography>
                  <SendableChooser ntKey="/SmartDashboard/autoChooser" />
                </Card>
              </Grid>

              {/* Odometry */}
              <Grid size={{ xs: 12, lg: 6 }}>
                <Card sx={{ p: 2, height: '100%', background: gradients.glass }}>
                  <Typography variant="h6" sx={{ mb: 1, fontSize: '0.95rem' }}>Odometry</Typography>
                  <Box sx={{
                    width: '100%',
                    background: 'rgba(0,0,0,0.4)',
                    borderRadius: '16px',
                    overflow: 'hidden',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    boxShadow: 'inset 0 4px 15px rgba(0,0,0,0.5)',
                    p: 0.5,
                  }}>
                    <Field
                      game="Rebuilt"
                      rotationUnit="deg"
                      style={{ width: '100%', height: '300px' }}
                    >
                      <FieldRobot
                        pose={robotPose ?? [0, 0, 0]}
                        width={0.85}
                        length={0.85}
                        color={isRedAlliance ? "#ff4444" : "#4488ff"}
                      />
                      <FieldPath
                        poses={[1, 1, 0, 2, 2, 0, 2, 5, 0]}
                      />
                    </Field>
                  </Box>
                  <Stack direction="row" spacing={2} sx={{ mt: 1 }}>
                    <Typography variant="subtitle2" sx={{ fontSize: '0.7rem' }}>X: {(robotPose?.[0] ?? 0).toFixed(2)} m</Typography>
                    <Typography variant="subtitle2" sx={{ fontSize: '0.7rem' }}>Y: {(robotPose?.[1] ?? 0).toFixed(2)} m</Typography>
                    <Typography variant="subtitle2" sx={{ fontSize: '0.7rem' }}>θ: {(robotPose?.[2] ?? 0).toFixed(1)}°</Typography>
                  </Stack>
                </Card>
              </Grid>
            </Grid>
          </Stack>
        )}

        {/* ── TELEOP TAB ── */}
        {activeTab === "teleop" && (
          <Stack spacing={1.5}>

            {/* Shift Timeline */}
            <Card sx={{ p: 2, background: gradients.glass }}>
              <ShiftTimeline
                totalDuration={MATCH_DURATION}
                totalRemaining={totalRemaining}
                currentShift={currentShift ?? ""}
                isAllianceActive={isAllianceActive ?? false}
                shifts={shifts}
              />
            </Card>

            {/* Status Cards */}
            <Grid container spacing={1.5}>
              <Grid size={{ xs: 12, sm: 6, md: 6 }}>
                <StatCard title="Camera Status" bgGradient={camerasConnected ? gradients.green : gradients.red}>
                  <Typography variant="h6" sx={{ fontSize: '1rem' }}>{camerasConnected ? "ONLINE" : "OFFLINE"}</Typography>
                </StatCard>
              </Grid>
              <Grid size={{ xs: 12, sm: 6, md: 6 }}>
                <StatCard title="Twindexer" bgGradient={twindexerJammed ? gradients.warning : gradients.glass}>
                  <Typography variant="h6" sx={{ fontSize: '1rem', color: twindexerJammed ? '#000' : '#fff', textShadow: twindexerJammed ? 'none' : undefined }}>
                    {twindexerJammed ? "JAMMED" : "CLEAR"}
                  </Typography>
                </StatCard>
              </Grid>
            </Grid>

            {/* Camera Feed */}
            <Card sx={{ p: 2, display: 'flex', flexDirection: 'column', background: gradients.glass }}>
              <Typography variant="h6" sx={{ mb: 1, fontSize: '0.95rem' }}>Camera Feed</Typography>
              <Box sx={{
                flexGrow: 1,
                minHeight: 280,
                background: 'rgba(0,0,0,0.5)',
                borderRadius: '16px',
                overflow: 'hidden',
                mb: 1.5,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                boxShadow: 'inset 0 4px 15px rgba(0,0,0,0.5)'
              }}>
                <Canvas>
                  <CanvasMjpgStream
                    srcs={streams}
                    width={320}
                    height={160}
                  // fps={fps}
                  // quality={quality}
                  />
                </Canvas>
              </Box>

              <Grid container spacing={2} sx={{ px: 1 }}>
                <Grid size={6}>
                  <Typography variant="subtitle2" gutterBottom sx={{ fontSize: '0.7rem' }}>Stream Quality: {quality}%</Typography>
                  <Slider value={quality} onChange={(_, val) => setQuality(val as number)} min={0} max={100} sx={{ color: KWARQS_YELLOW, height: 6 }} />
                </Grid>
                <Grid size={6}>
                  <Typography variant="subtitle2" gutterBottom sx={{ fontSize: '0.7rem' }}>Target FPS: {fps}</Typography>
                  <Slider value={fps} onChange={(_, val) => setFps(val as number)} min={1} max={60} sx={{ color: KWARQS_GREEN, height: 6 }} />
                </Grid>
              </Grid>
            </Card>
          </Stack>
        )}

      </Container>
    </ThemeProvider>
  );
}

export default App;