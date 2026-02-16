import { create } from 'zustand';
import { immer } from 'zustand/middleware/immer';
import { NT4Provider } from './nt4/NT4Provider';

// Topic state interface
interface TopicState {
  value: unknown;
  type: string;
  timestamp?: number;
}

// Store state interface
interface Nt4State {
  // NT4Provider instance
  provider: NT4Provider;

  // Connection state
  isConnected: boolean;
  address: string;

  // Topics state - maps topic name to its current state
  topics: Record<string, TopicState>;

  // Actions
  connect: (address: string) => void;
  disconnect: () => void;
  updateTopic: (key: string, value: unknown, type: string) => void;
}

// Create the vanilla store
const useNt4Store = create<Nt4State>()(
  immer((set, get) => {
    // Create NT4Provider instance
    const provider = new NT4Provider();

    // Listen for connection status changes
    provider.on('connectionStatusChanged', (connected: boolean, _label: string) => {
      set((state) => {
        state.isConnected = connected;
      });
    });

    // Listen for topic updates
    provider.on('sourcesChanged', (sources: Array<{ key: string; value: unknown; type: string }>) => {
      set((state) => {
        sources.forEach(({ key, value, type }) => {
          state.topics[key] = {
            value,
            type,
            timestamp: Date.now(),
          };
        });
      });
    });

    return {
      provider,
      isConnected: false,
      address: 'localhost',
      topics: {},

      // Connect to a NetworkTables server
      connect: (address: string) => {
        get().provider.connect(address);
        set((state) => {
          state.address = address;
        });
      },

      // Disconnect from the server
      disconnect: () => {
        get().provider.disconnect();
        set((state) => {
          state.isConnected = false;
        });
      },
      updateTopic: (key: string, value: unknown, type: string) => {
        get().provider.componentUpdate(key, value, type);
      },
    };
  })
);

export default useNt4Store;