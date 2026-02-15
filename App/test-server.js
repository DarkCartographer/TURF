const express = require('express');
const cors = require('cors');
const app = express();

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
  nextChargeMinutes: 45,
  position: null
};

let fieldConfig = null;
// Mower starts at origin (0,0)
let mowerPosition = { x: 0.5, y: 0.5 }; // Start near origin in meters

// Log all requests
app.use((req, res, next) => {
  console.log(`[${new Date().toLocaleTimeString()}] ${req.method} ${req.url} from ${req.ip}`);
  next();
});

// Simulate mower movement in a grid pattern
setInterval(() => {
  if (mowerState.isMowing && fieldConfig) {
    const width = parseFloat(fieldConfig.dimensions.width);
    const height = parseFloat(fieldConfig.dimensions.height);
    
    // Move in a lawnmower pattern (back and forth)
    mowerPosition.x += 0.3;
    
    // When reaching edge, move down and reverse direction
    if (mowerPosition.x >= width - 0.5) {
      mowerPosition.x = width - 0.5;
      mowerPosition.y += 0.5;
      if (mowerPosition.y >= height - 0.5) {
        mowerPosition.y = height - 0.5;
      }
    }
    
    // Keep within bounds
    mowerPosition.x = Math.max(0, Math.min(width, mowerPosition.x));
    mowerPosition.y = Math.max(0, Math.min(height, mowerPosition.y));
  }
}, 2000);

app.get('/api/status', (req, res) => {
  console.log('Status requested');
  
  if (mowerState.isMowing && mowerState.mowingProgress < 100) {
    mowerState.mowingProgress += 1;
    mowerState.mowedArea += 4;
    mowerState.remainingArea = Math.max(0, mowerState.remainingArea - 4);
    mowerState.batteryLevel = Math.max(0, mowerState.batteryLevel - 0.25);
  }

  // Simulate position data
  if (fieldConfig) {
    const width = parseFloat(fieldConfig.dimensions.width);
    const height = parseFloat(fieldConfig.dimensions.height);

    mowerState.position = {
      x: parseFloat(mowerPosition.x.toFixed(2)),
      y: parseFloat(mowerPosition.y.toFixed(2))
    };

    // Simulate occasional position warning for testing
    const simulateWarning = mowerState.mowingProgress % 20 === 0;
    const errorAmount = simulateWarning ? 1.5 : 0.05;

    mowerState.positionDebug = {
      x_deadReckoning: parseFloat((mowerPosition.x + errorAmount * 0.3).toFixed(3)),
      y_deadReckoning: parseFloat((mowerPosition.y + errorAmount * 0.3).toFixed(3)),
      x_triangulated: parseFloat(mowerPosition.x.toFixed(3)),
      y_triangulated: parseFloat(mowerPosition.y.toFixed(3)),
      positionError: parseFloat(errorAmount.toFixed(3)),
      positionWarning: errorAmount > 0.5,
      triangulationValid: true
    };
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

app.post('/api/field-config', (req, res) => {
  console.log('Field configuration received:', req.body);
  const { anchors, dimensions } = req.body;
  
  // Validate
  if (!anchors || anchors.length !== 3) {
    return res.status(400).json({ 
      success: false, 
      message: 'Exactly 3 anchors required' 
    });
  }

  if (!dimensions || !dimensions.width || !dimensions.height) {
    return res.status(400).json({
      success: false,
      message: 'Field dimensions required'
    });
  }

  // Validate anchor positions
  const anchor1 = anchors.find(a => a.corner === 'TL');
  const anchor2 = anchors.find(a => a.corner === 'TR');
  const anchor3 = anchors.find(a => a.corner === 'BL');

  const width = parseFloat(dimensions.width);
  const height = parseFloat(dimensions.height);

  fieldConfig = { anchors, dimensions };
  
  // Reset mower to starting position (0,0)
  mowerPosition = { x: 0.5, y: 0.5 };
  
  console.log('Field configured successfully');
  console.log(`Field size: ${width}m x ${height}m`);
  console.log(`Mower starting at: (${mowerPosition.x}, ${mowerPosition.y})`);
  
  res.json({ 
    success: true, 
    message: `Field configured: ${width}m x ${height}m. Mower positioned at origin.`,
    config: fieldConfig
  });
});

const PORT = 3000;
app.listen(PORT, '0.0.0.0', () => {
  console.log('========================================');
  console.log(`Test server running on port ${PORT}`);
  console.log(`Listening on: 0.0.0.0`);
  console.log('========================================');
  console.log('Mower uses grid coordinate system:');
  console.log('  Origin (0,0) = Top-Left corner');
  console.log('  X-axis = Width (left to right)');
  console.log('  Y-axis = Height (top to bottom)');
  console.log('========================================');
});
