import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import App from './App.tsx'
import {SomeComponent} from './SomeComponent.tsx';

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
    <SomeComponent name="Toba" />
  </StrictMode>,
)
