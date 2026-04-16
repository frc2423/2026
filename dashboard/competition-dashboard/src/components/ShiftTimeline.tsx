import { keyframes } from '@emotion/react';
import { Box, Card, Typography } from '@mui/material';

export interface Shift {
  name: string;
  startSec: number;
  endSec: number;
  alliance: 'red' | 'blue' | 'both';
}

export interface ShiftTimelineProps {
  totalDuration: number;
  totalRemaining: number;
  currentShift: string;
  isAllianceActive: boolean;
  shifts: Shift[];
}

const pulseGlow = keyframes`
  0%, 100% { filter: brightness(1.15); }
  50%       { filter: brightness(1.6);  }
`;

const ALLIANCE_STYLE: Record<Shift['alliance'], { bg: string; shadow: string }> = {
  red:  { bg: 'linear-gradient(180deg, #c0003a 0%, #ff2255 100%)', shadow: 'rgba(255,34,85,0.7)'  },
  blue: { bg: 'linear-gradient(180deg, #0030b0 0%, #2255ff 100%)', shadow: 'rgba(34,85,255,0.7)'  },
  both: { bg: 'linear-gradient(180deg, #5500bb 0%, #aa33ff 100%)', shadow: 'rgba(150,50,255,0.7)' },
};

function fmt(sec: number): string {
  if (sec < 0) return '--:--';
  const m = Math.floor(sec / 60);
  const s = Math.floor(sec % 60);
  return `${m}:${s.toString().padStart(2, '0')}`;
}

export default function ShiftTimeline({
  totalDuration,
  totalRemaining,
  currentShift,
  isAllianceActive,
  shifts,
}: ShiftTimelineProps) {
  const elapsed =
    totalRemaining < 0
      ? -1
      : Math.max(0, Math.min(totalDuration, totalDuration - totalRemaining));
  const cursorPct = elapsed < 0 ? null : (elapsed / totalDuration) * 100;

  return (
    <Box sx={{ display: 'flex', gap: 2, alignItems: 'stretch', height: '100%' }}>

      {/* ── Alliance Active Button ── */}
      <Card
        sx={{
          minWidth: 130,
          flexShrink: 0,
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          p: 2,
          cursor: 'default',
          userSelect: 'none',
          background: isAllianceActive
            ? 'linear-gradient(135deg, #0ba360 0%, #3cba92 100%)'
            : 'linear-gradient(135deg, #ff0844 0%, #ffb199 100%)',
          boxShadow: isAllianceActive
            ? '0 0 28px rgba(11,163,96,0.65), 0 8px 32px rgba(0,0,0,0.4)'
            : '0 0 28px rgba(255,8,68,0.65),  0 8px 32px rgba(0,0,0,0.4)',
          animation: `${pulseGlow} 2s ease-in-out infinite`,
        }}
      >
        <Typography sx={{ fontSize: '2.4rem', lineHeight: 1, mb: 0.5 }}>
          {isAllianceActive ? '✓' : '✗'}
        </Typography>
        <Typography
          sx={{
            fontWeight: 900,
            fontSize: '0.9rem',
            textTransform: 'uppercase',
            letterSpacing: '2px',
            textShadow: '0 2px 8px rgba(0,0,0,0.4)',
          }}
        >
          {isAllianceActive ? 'ACTIVE' : 'INACTIVE'}
        </Typography>
      </Card>

      {/* ── Timeline ── */}
      <Box sx={{ flex: 1, display: 'flex', flexDirection: 'column', justifyContent: 'center', gap: 0.5 }}>

        {/* Floating time label (moves with cursor) */}
        <Box sx={{ position: 'relative', height: 22 }}>
          {cursorPct !== null && (
            <Typography
              sx={{
                position: 'absolute',
                left: `${Math.min(Math.max(cursorPct, 4), 96)}%`,
                transform: 'translateX(-50%)',
                fontSize: '0.85rem',
                fontWeight: 800,
                letterSpacing: '1.5px',
                whiteSpace: 'nowrap',
                textShadow: '0 2px 6px rgba(0,0,0,0.7)',
                pointerEvents: 'none',
              }}
            >
              {fmt(totalRemaining)}
            </Typography>
          )}
        </Box>

        {/* Segmented bar */}
        <Box
          sx={{
            position: 'relative',
            height: 56,
            borderRadius: '12px',
            overflow: 'hidden',
            display: 'flex',
            boxShadow: '0 4px 20px rgba(0,0,0,0.5)',
          }}
        >
          {shifts.map((shift, i) => {
            const widthPct = ((shift.endSec - shift.startSec) / totalDuration) * 100;
            const isCurrent = shift.name === currentShift;
            const { bg, shadow } = ALLIANCE_STYLE[shift.alliance];
            return (
              <Box
                key={i}
                sx={{
                  width: `${widthPct}%`,
                  background: bg,
                  position: 'relative',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  borderRight: '1px solid rgba(255,255,255,0.12)',
                  transition: 'filter 0.4s ease, box-shadow 0.4s ease',
                  filter: isCurrent ? 'brightness(1.15)' : 'brightness(0.72)',
                  boxShadow: isCurrent ? `inset 0 0 24px ${shadow}` : 'none',
                  animation: isCurrent ? `${pulseGlow} 1.8s ease-in-out infinite` : 'none',
                }}
              >
                <Typography
                  sx={{
                    fontSize: '0.6rem',
                    fontWeight: 700,
                    textShadow: '0 1px 4px rgba(0,0,0,0.8)',
                    px: 0.25,
                    overflow: 'hidden',
                    textOverflow: 'ellipsis',
                    whiteSpace: 'nowrap',
                    maxWidth: '92%',
                    textAlign: 'center',
                  }}
                >
                  {shift.name}
                </Typography>
              </Box>
            );
          })}

          {/* Cursor */}
          {cursorPct !== null && (
            <Box
              sx={{
                position: 'absolute',
                left: `${cursorPct}%`,
                top: 0,
                bottom: 0,
                width: 3,
                transform: 'translateX(-50%)',
                background: 'white',
                boxShadow: '0 0 10px rgba(255,255,255,0.95), 0 0 22px rgba(255,255,255,0.4)',
                borderRadius: '2px',
                zIndex: 10,
                pointerEvents: 'none',
              }}
            />
          )}
        </Box>

        {/* Tick marks — start times */}
        <Box sx={{ display: 'flex' }}>
          {shifts.map((shift, i) => {
            const widthPct = ((shift.endSec - shift.startSec) / totalDuration) * 100;
            return (
              <Box key={i} sx={{ width: `${widthPct}%`, display: 'flex', justifyContent: 'flex-start', pl: 0.5 }}>
                <Typography sx={{ fontSize: '0.5rem', opacity: 0.4, whiteSpace: 'nowrap' }}>
                  {shift.startSec}s
                </Typography>
              </Box>
            );
          })}
        </Box>

      </Box>
    </Box>
  );
}
