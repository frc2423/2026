import React, { useState, type FormEvent } from "react";
import { useNTValue, useNTConnection } from "./store/useNetworktables";
import { BooleanBox, Canvas, CanvasMjpgStream } from "@frc-web-components/react";
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
  TextField,
  Typography,
  ThemeProvider,
  createTheme,
  CssBaseline
} from "@mui/material";
import NumberField from "./components/NumberField";

// 1. Bubbly, Vibrant Theme
const bubblyTheme = createTheme({
  palette: {
    mode: 'dark',
    primary: { main: '#a855f7' }, // Fun purple accent
    background: { default: '#0f172a', paper: 'rgba(30, 41, 59, 0.6)' }, // Deep navy glass
  },
  shape: { borderRadius: 32 }, // Maximum "bubbly" pill shapes
  typography: {
    fontFamily: '"Nunito", "Inter", sans-serif', // Nunito is a highly readable, rounded/bubbly font
    h4: { fontWeight: 800, letterSpacing: '-0.5px' },
    h6: { fontWeight: 700 },
    subtitle2: { textTransform: 'uppercase', letterSpacing: '1.5px', fontSize: '0.8rem', opacity: 0.9, fontWeight: 700 }
  },
  components: {
    MuiCssBaseline: {
      styleOverrides: {
        body: {
          // A deep, vibrant mesh gradient background for the whole app
          background: 'linear-gradient(135deg, #1e1b4b 0%, #312e81 50%, #4c1d95 100%)',
          backgroundAttachment: 'fixed',
          minHeight: '100vh',
        }
      }
    },
    MuiCard: {
      styleOverrides: {
        root: {
          backdropFilter: 'blur(16px)', // Glassmorphism
          border: '2px solid rgba(255, 255, 255, 0.1)',
          boxShadow: '0 8px 32px rgba(0, 0, 0, 0.3)',
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
    }
  }
});

// Vibrant Gradients for Utility States
const gradients = {
  glass: 'rgba(255, 255, 255, 0.05)',
  red: 'linear-gradient(135deg, #ff0844 0%, #ffb199 100%)',
  blue: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)',
  green: 'linear-gradient(135deg, #0ba360 0%, #3cba92 100%)',
  warning: 'linear-gradient(135deg, #f6d365 0%, #fda085 100%)',
};

// 2. Bubbly StatCard: Takes a gradient background for active states
const StatCard = ({ title, children, bgGradient = gradients.glass }: { title: string, children: React.ReactNode, bgGradient?: string }) => (
  <Card sx={{ 
    height: "100%", p: 3, display: "flex", flexDirection: "column", justifyContent: "space-between",
    background: bgGradient,
    transition: 'transform 0.2s ease-in-out',
    '&:hover': { transform: 'translateY(-4px)' }
  }}>
    <Typography variant="subtitle2" sx={{ mb: 2, textShadow: '0 2px 4px rgba(0,0,0,0.3)' }}>{title}</Typography>
    <Box sx={{ display: "flex", alignItems: "center", textShadow: '0 2px 10px rgba(0,0,0,0.4)' }}>
      {children}
    </Box>
  </Card>
);

function App() {
  const { isConnected, address, connect } = useNTConnection();
  const [addressInput, setAddressInput] = useState(address);

  const [someStringArray, setSomeStringArray] = useNTValue<string[]>("/SmartDashboard/someStringArray");
  const [someNumberArray, setSomeNumberArray] = useNTValue<number[]>("/SmartDashboard/setSomeNumberArray");

  const handleSubmit = (e: FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    const formData = new FormData(e.currentTarget);
    const stepNames = formData.getAll("stepName") as string[];
    const stepDelays = formData.getAll("stepDelay").map((val) => parseFloat(val as string));

    setSomeStringArray(stepNames);
    setSomeNumberArray(stepDelays);
  };

  const [isRedAlliance] = useNTValue<boolean>("/FMSInfo/IsRedAlliance", false);
  const [streams] = useNTValue<string[]>("/CameraPublisher/USB Camera 0/streams", []);
  const [quality, setQuality] = useState(50);
  const [fps, setFps] = useState(60);

  const [activeAlliance] = useNTValue<string>("/Robot/robotContainer/dashboardlogger/activeAlliance", "No Data");
  const [currentShift] = useNTValue<string>("/Robot/robotContainer/dashboardlogger/currentShift", "No Data");
  const [isAllianceActive] = useNTValue<boolean>("/Robot/robotContainer/dashboardlogger/isAllianceActive", true);
  const [isAllianceActiveNextShift] = useNTValue<boolean>("/Robot/robotContainer/dashboardlogger/isAllianceActiveNextShift", true);
  const [camerasConnected] = useNTValue<boolean>("/visionDebug/april_tag_cam/camerasConnected", true);
  const [twindexerJammed] = useNTValue<boolean>("/Robot/robotContainer/twindexer/isJammed", false);

  const allianceGradient = activeAlliance === "both" ? gradients.glass : activeAlliance === "red" ? gradients.red : gradients.blue;

  return (
    <ThemeProvider theme={bubblyTheme}>
      <CssBaseline />
      <Container maxWidth={false} sx={{ py: 4, px: { xs: 2, md: 4 } }}>
        
        {/* Top Navigation / Status Bar */}
        <Stack spacing={3} sx={{ mb: 4, flexDirection: { xs: 'column', md: 'row' }, justifyContent: 'space-between', alignItems: 'center' }}>
          <Card sx={{ p: 1.5, px: 3, display: 'flex', alignItems: 'center', gap: 2, width: { xs: '100%', md: '500px' }, background: gradients.glass }}>
            <Box sx={{ 
              width: 18, height: 18, borderRadius: '50%', 
              background: isConnected ? gradients.green : gradients.red,
              boxShadow: isConnected ? '0 0 15px #0ba360' : '0 0 15px #ff0844'
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
              sx={{ '& .MuiOutlinedInput-notchedOutline': { border: 'none' } }}
            />
          </Card>

          <Card sx={{ p: 2, px: 4, background: isRedAlliance ? gradients.red : gradients.blue }}>
            <Typography variant="h6" sx={{ textTransform: 'uppercase', textShadow: '0 2px 5px rgba(0,0,0,0.4)' }}>
              {isRedAlliance ? "Red Alliance" : "Blue Alliance"}
            </Typography>
          </Card>
        </Stack>

        {/* Telemetry Dashboard - VIBRANT UTILITY BLOCKS */}
        <Grid container spacing={3} sx={{ mb: 4 }}>
          <Grid size={{ xs: 12, sm: 6, md: 2 }}>
            <StatCard title="Active Alliance" bgGradient={allianceGradient}>
              <Typography variant="h4" sx={{ textTransform: 'uppercase' }}>{activeAlliance}</Typography>
            </StatCard>
          </Grid>
          <Grid size={{ xs: 12, sm: 6, md: 2 }}>
            <StatCard title="Current Shift">
              <Typography variant="h6" sx={{ opacity: 0.9 }}>{currentShift}</Typography>
            </StatCard>
          </Grid>
          <Grid size={{ xs: 12, sm: 6, md: 2 }}>
            <StatCard title="Alliance Active" bgGradient={isAllianceActive ? gradients.green : gradients.red}>
              <Typography variant="h4">{isAllianceActive ? "YES" : "NO"}</Typography>
            </StatCard>
          </Grid>
          <Grid size={{ xs: 12, sm: 6, md: 2 }}>
            <StatCard title="Active Next Shift" bgGradient={isAllianceActiveNextShift ? gradients.green : gradients.red}>
              <Typography variant="h4">{isAllianceActiveNextShift ? "YES" : "NO"}</Typography>
            </StatCard>
          </Grid>
          <Grid size={{ xs: 12, sm: 6, md: 2 }}>
            <StatCard title="Camera Status" bgGradient={camerasConnected ? gradients.green : gradients.red}>
              <Typography variant="h4">{camerasConnected ? "ONLINE" : "OFFLINE"}</Typography>
            </StatCard>
          </Grid>
          <Grid size={{ xs: 12, sm: 6, md: 2 }}>
            {/* Extremely obvious warning if jammed */}
            <StatCard title="Twindexer" bgGradient={twindexerJammed ? gradients.warning : gradients.glass}>
              <Typography variant="h4" sx={{ color: twindexerJammed ? '#000' : '#fff', textShadow: twindexerJammed ? 'none' : undefined }}>
                {twindexerJammed ? "JAMMED" : "CLEAR"}
              </Typography>
            </StatCard>
          </Grid>
        </Grid>

        {/* Lower Section: Controls & Camera Feed */}
        <Grid container spacing={4}>
          
          {/* Auto Routine Form */}
          <Grid size={{ xs: 12, lg: 5 }}>
            <Card sx={{ p: 4, height: '100%', background: gradients.glass }}>
              <Typography variant="h6" sx={{ mb: 4 }}>Autonomous Routine Setup</Typography>
              <form onSubmit={handleSubmit}>
                <Stack spacing={4}>
                  
                  {/* Step 1 */}
                  <Stack direction="row" spacing={2}>
                    <FormControl fullWidth>
                      <InputLabel>Step 1 Action</InputLabel>
                      <Select name="stepName" label="Step 1 Action" defaultValue="none" sx={{ borderRadius: '20px' }}>
                        <MenuItem value="shoot">Shoot</MenuItem>
                        <MenuItem value="brief">Brief Delay</MenuItem>
                        <MenuItem value="outpost or depot">Go to Outpost/Depot</MenuItem>
                        <MenuItem value="none">None</MenuItem>
                      </Select>
                    </FormControl>
                    <Box sx={{ width: 150 }}><NumberField name="stepDelay" label="Delay (s)" min={0} max={20} defaultValue={0} /></Box>
                  </Stack>

                  {/* Step 2 */}
                  <Stack direction="row" spacing={2}>
                    <FormControl fullWidth>
                      <InputLabel>Step 2 Action</InputLabel>
                      <Select name="stepName" label="Step 2 Action" defaultValue="trench" sx={{ borderRadius: '20px' }}>
                        <MenuItem value="trench">Go Under Trench</MenuItem>
                        <MenuItem value="bump">Go Over Bump</MenuItem>
                      </Select>
                    </FormControl>
                    <Box sx={{ width: 150 }}><NumberField name="stepDelay" label="Delay (s)" min={0} max={20} defaultValue={14} /></Box>
                  </Stack>

                  {/* Step 3 */}
                  <Stack direction="row" spacing={2}>
                    <FormControl fullWidth>
                      <InputLabel>Step 3 Action</InputLabel>
                      <Select name="stepName" label="Step 3 Action" defaultValue="bump" sx={{ borderRadius: '20px' }}>
                        <MenuItem value="trench">Go Under Trench</MenuItem>
                        <MenuItem value="bump">Go Over Bump</MenuItem>
                        <MenuItem value="outpost or depot">Go to Outpost/Depot</MenuItem>
                        <MenuItem value="shoot">Shoot</MenuItem>
                        <MenuItem value="collect">Collect Fuel</MenuItem>
                      </Select>
                    </FormControl>
                  </Stack>

                  <Box sx={{ pt: 2 }}>
                    <Button type="submit" variant="contained" fullWidth sx={{ 
                      background: 'linear-gradient(135deg, #a855f7 0%, #ec4899 100%)',
                      color: 'white',
                      '&:hover': { background: 'linear-gradient(135deg, #9333ea 0%, #db2777 100%)' }
                    }}>
                      Deploy Routine
                    </Button>
                  </Box>
                </Stack>
              </form>
            </Card>
          </Grid>

          {/* Camera Feed & Settings */}
          <Grid size={{ xs: 12, lg: 7 }}>
            <Card sx={{ p: 3, display: 'flex', flexDirection: 'column', height: '100%', background: gradients.glass }}>
              <Box sx={{ 
                flexGrow: 1, 
                minHeight: 360, 
                background: 'rgba(0,0,0,0.5)', 
                borderRadius: '24px', 
                overflow: 'hidden', 
                mb: 3,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                boxShadow: 'inset 0 4px 15px rgba(0,0,0,0.5)'
              }}>
                <Canvas>
                  <CanvasMjpgStream
                    srcs={streams}
                    resolutionWidth={320}
                    resolutionHeight={160}
                    fps={fps}
                    quality={quality}
                  />
                </Canvas>
              </Box>
              
              <Grid container spacing={4} sx={{ px: 2, pb: 1 }}>
                <Grid size={6}>
                  <Typography variant="subtitle2" gutterBottom>
                    Stream Quality: {quality}%
                  </Typography>
                  <Slider value={quality} onChange={(_, val) => setQuality(val as number)} min={0} max={100} sx={{ color: '#ec4899', height: 8 }} />
                </Grid>
                <Grid size={6}>
                  <Typography variant="subtitle2" gutterBottom>
                    Target FPS: {fps}
                  </Typography>
                  <Slider value={fps} onChange={(_, val) => setFps(val as number)} min={1} max={60} sx={{ color: '#a855f7', height: 8 }} />
                </Grid>
              </Grid>
            </Card>
          </Grid>

        </Grid>
      </Container>
    </ThemeProvider>
  );
}

export default App;