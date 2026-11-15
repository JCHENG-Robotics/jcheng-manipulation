import { Navigate, Route, Routes } from "react-router-dom";
import { AppShell } from "./components/AppShell";
import { useRobotStream } from "./hooks/useRobotStream";
import { Dashboard } from "./pages/Dashboard";
import { DiagnosticsPage } from "./pages/Diagnostics";
import { EventTimeline } from "./pages/EventTimeline";
import { JointControl } from "./pages/JointControl";
import { SettingsPage } from "./pages/Settings";

export default function App() {
  useRobotStream();
  return (
    <Routes>
      <Route element={<AppShell />}>
        <Route index element={<Dashboard />} />
        <Route path="joints" element={<JointControl />} />
        <Route path="diagnostics" element={<DiagnosticsPage />} />
        <Route path="events" element={<EventTimeline />} />
        <Route path="settings" element={<SettingsPage />} />
        <Route path="*" element={<Navigate to="/" replace />} />
      </Route>
    </Routes>
  );
}
