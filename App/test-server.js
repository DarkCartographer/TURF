const express = require('express');
const cors = require('cors');
const app = express();

// Enable CORS for all origins
app.use(cors({
  origin: '*',
  methods: ['GET', 'POST', 'OPTIONS'],
  allowedHeaders: ['Content-Type']
}));

app.use(express.json());

let mowerState = {
  isMowing: true,
  batteryLevel: 75,
  batteryMinutes: 90,
  mowingProgress: 42,
  mowedArea: 168,
  remainingArea: 232,
  totalArea: 400,
  totalDistance: 2.4,
  runtimeHours: 1,
  runtimeMinutes: 24,
  nextChargeMinutes: 45
};

// Log all requests
app.use((req, res, next) => {
  console.log(`[${new Date().toLocaleTimeString()}] ${req.method} ${req.url} from ${req.ip}`);
  next();
});

app.get('/api/status', (req, res) => {
  console.log('Status requested');
  if (mowerState.isMowing && mowerState.mowingProgress < 100) {
    mowerState.mowingProgress += 1;
    mowerState.mowedArea += 4;
    mowerState.remainingArea = Math.max(0, mowerState.remainingArea - 4);
    mowerState.batteryLevel = Math.max(0, mowerState.batteryLevel - 0.25);
  }
  res.json(mowerState);
});

app.post('/api/command', (req, res) => {
  console.log('Command received:', req.body);
  const { command } = req.body;
  if (command === 'emergency_stop') {
    mowerState.isMowing = false;
    res.json({ success: true, message: 'Emergency stop executed' });
  } else if (command === 'pause') {
    mowerState.isMowing = false;
    res.json({ success: true, message: 'Mowing paused' });
  } else if (command === 'resume') {
    mowerState.isMowing = true;
    res.json({ success: true, message: 'Mowing resumed' });
  } else {
    res.status(400).json({ success: false, message: 'Unknown command' });
  }
});

app.post('/api/pattern', (req, res) => {
  console.log('Pattern uploaded');
  res.json({ success: true, message: 'Pattern uploaded' });
});

const PORT = 3000;
const HOST = '0.0.0.0';

app.listen(PORT, HOST, () => {
  console.log('========================================');
  console.log(`Test server running on port ${PORT}`);
  console.log(`Listening on: ${HOST}`);
  console.log(`Local: http://localhost:${PORT}`);
  console.log(`Network: http://192.168.0.104:${PORT}`);
  console.log('========================================');
  console.log('Try these URLs:');
  console.log(`  Computer: http://localhost:${PORT}/api/status`);
  console.log(`  Phone: http://192.168.0.104:${PORT}/api/status`);
  console.log('========================================');
});