import useNt4Store from './useNt4Store';

/**
 * Hook to monitor the WebSocket connection status
 */
export function useNTConnection() {
    const isConnected = useNt4Store(state => state.isConnected);
    const address = useNt4Store(state => state.address);
    const connect = useNt4Store(state => state.connect);
    const disconnect = useNt4Store(state => state.disconnect);
    return { isConnected, address,connect, disconnect };
}


/**
 * Hook to get and set a NetworkTables value
 */
export function useNTValue<T>(key: string, defaultValue?: T): [T | undefined, (value: T) => void] {
    const value = useNt4Store(state => state.topics[key]?.value as T | undefined);
    const updateTopic = useNt4Store(state => state.updateTopic);

    const setValue = (newValue: T) => {
        // Infer type from the current topic or use a default type
        const currentTopic = useNt4Store.getState().topics[key];
        const type = currentTopic?.type || 'string';
        updateTopic(key, newValue, type);
    };

    return [value ?? defaultValue, setValue];
}


/**
 * Hook to get all current NetworkTables keys
 */
export function useNTKeys(): string[] {
    const keys = useNt4Store(state => Object.keys(state.topics));
    return keys;
}

/**
 * Hook to check if a key exists in NetworkTables
 */
export function useNTKeyExists(key: string): boolean {
    const exists = useNt4Store(state => key in state.topics);
    return exists;
}