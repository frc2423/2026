import { createStore } from 'zustand'
import { immer } from 'zustand/middleware/immer';

interface Nt4State {

}

export const useNt4Store = () => createStore<Nt4State>()(
  immer((set) => ({
    
  }))
);