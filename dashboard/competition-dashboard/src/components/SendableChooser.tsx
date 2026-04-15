import { Autocomplete, TextField } from '@mui/material';
import { useNTValue } from '../store/useNetworktables';

interface Props {
  ntKey: string;
}

export default function SendableChooser({ ntKey }: Props) {
  const [options] = useNTValue<string[]>(`${ntKey}/options`, []);
  const [selected, setSelected] = useNTValue<string>(`${ntKey}/selected`, "");
  const [defaultValue] = useNTValue<string>(`${ntKey}/default`, "");
  const [active] = useNTValue<string>(`${ntKey}/active`, "");
  const [label] = useNTValue<string>(`${ntKey}/label`, "Auto Choices");

  return (
    <Autocomplete
      fullWidth
      size="small"
      onChange={(_, newValue) => {
        if (newValue !== null) {
          setSelected(newValue);
        }
      }}
      options={options ?? []}
      value={selected || active || ''}
      defaultValue={defaultValue ?? ''}
      disableClearable
      renderInput={(params) => (
        <TextField
          {...params}
          error={selected !== active}
          label={label}
          variant="outlined"
          sx={{
            '& .MuiOutlinedInput-root': {
              borderRadius: '16px',
              backgroundColor: 'rgba(255,255,255,0.08)',
            },
          }}
        />
      )}
    />
  );
}
