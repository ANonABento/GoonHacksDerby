import React, { useState, useEffect, useRef, useCallback } from 'react';

// --- CONFIGURATION CONSTANTS ---
const ESP32_IP_ADDRESS = 'IP_ADDRESS_HERE'; 
const API_LOSE_ENDPOINT = `http://${ESP32_IP_ADDRESS}/lose`;

const RACER_SIZE = 3;
const TAIL_LENGTH = 40;
const TRACK_COLOR = '#3a3a3a';
const RACER_WIDTH = 5;
const RACER_HEIGHT = 3;

const OBSTACLE_COUNT = 1; // Single giant obstacle
const OBSTACLE_SIZE_MIN = 600; // Minimum size for giant center obstacle
const OBSTACLE_SIZE_MAX = 800; // Maximum size for giant center obstacle
const OBSTACLE_REPULSION = 0.25;
const OBSTACLE_ROTATION_SPEED = 0.002; // Much slower rotation (degrees per frame)
const OBSTACLE_ORBIT_RADIUS = 30; // Smaller radius for flowing/orbital movement
const OBSTACLE_ORBIT_SPEED = 0.001; // Very slow orbital movement (reduced from 0.003)
const BASE_VELOCITY = 0.8;
const MIN_EXPECTED_VELOCITY = BASE_VELOCITY * 0.3; // Minimum velocity racers should maintain (30% of base)
const STUCK_VELOCITY_THRESHOLD = MIN_EXPECTED_VELOCITY; // Use minimum expected velocity as threshold
const ESCAPE_FORCE = 2.0; // Force to apply when escaping
const STUCK_DURATION = 10; // Frames to stay stuck before intelligent escape (reduced)
const WIGGLE_STRENGTH = 0.3; // Strength of wiggling movement
const ESCAPE_BURST_VELOCITY = 3.0; // High velocity burst for intelligent escape
const ESCAPE_BURST_DURATION = 15; // Frames to maintain escape burst
const OBSTACLE_AVOIDANCE_DISTANCE = 500; // Distance to start detecting and avoiding obstacle (increased significantly)
const OBSTACLE_DECELERATION_DISTANCE = 400; // Distance to start decelerating
const STUCK_DETECTION_FRAMES = 10; // Frames without movement to consider stuck (reduced for faster response)
const STUCK_ESCAPE_FORCE = 2.5; // Force multiplier when stuck and escaping (increased)
const COLLISION_ESCAPE_FORCE = 1.8; // Force multiplier when colliding (applied immediately)
const COLLISION_ESCAPE_FRAMES = 5; // Frames of collision before applying aggressive escape
const OBSTACLE_AVOIDANCE_STRENGTH = 0.8; // Strength of avoidance force
const DECELERATION_FACTOR = 0.85; // How much to reduce velocity when approaching obstacle
const ACCELERATION_FACTOR = 1.15; // How much to increase velocity after avoiding (capped at BASE_VELOCITY)
const MIN_APPROACH_VELOCITY = BASE_VELOCITY * 0.4; // Minimum velocity when approaching obstacle
const STUCK_POSITION_THRESHOLD = 5; // Max position change to consider "stuck in place"
const RANDOM_FACTOR = 0.05;
const TARGET_SMOOTHING = 0.05;
const WALL_BOUNCE_DAMPING = 0.8;
const GOAL_PULL_STRENGTH = 0.03;
const DRAG = 0.997;
const VELOCITY_SMOOTHING = 0.08; // Smooth velocity changes (lower = smoother)
const PATH_PLANNING_LOOKAHEAD = 200; // How far ahead to plan path (pixels) - increased for earlier detection
const PATH_PLANNING_STEPS = 10; // Number of steps to check ahead - more steps for better prediction
const MIN_TURN_RATE = 0.02; // Minimum radians per frame for turning
const MAX_TURN_RATE = 0.08; // Maximum radians per frame for turning
const AVOIDANCE_LOOKAHEAD_TIME = 30; // Frames ahead to check for collisions
const SCREEN_EDGE_PENALTY_DISTANCE = 100; // Distance from screen edge to start penalizing paths

const HORSES_CONFIG = [
    { id: 1, name: "Red Shift", color: '#ff4d4f', emoji: '🔴' },
    { id: 2, name: "Blue Streak", color: '#1890ff', emoji: '🔵' },
    { id: 3, name: "Lime Ghost", color: '#a0d911', emoji: '🟢' },
    { id: 4, name: "Golden Gallop", color: '#ffc53d', emoji: '🟡' },
    { id: 5, name: "Cyan Comet", color: '#597ef7', emoji: '🧊' },
    { id: 6, name: "Violet Venom", color: '#722ed1', emoji: '🟣' },
    { id: 7, name: "Orange Fury", color: '#fa8c16', emoji: '🟠' },
    { id: 8, name: "Pink Phantom", color: '#eb2f96', emoji: '🌸' },
];

const generateDust = (width, height, count = 200) => {
    return Array.from({ length: count }, () => ({
        x: Math.random() * width,
        y: Math.random() * height,
        size: Math.random() * 2.5 + 0.5,
        opacity: Math.random() * 0.3 + 0.1
    }));
};

// Path planning: Find best direction to goal while avoiding obstacles
const planPath = (x, y, goalX, goalY, currentVx, currentVy, obstacle, racerRadius, lookaheadDistance, steps, screenWidth, screenHeight) => {
    if (!obstacle) {
        // No obstacle, direct path to goal
        const dx = goalX - x;
        const dy = goalY - y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 1) return { angle: Math.atan2(currentVy, currentVx), score: 1 };
        return { angle: Math.atan2(dy, dx), score: 1 };
    }
    
    const obstacleSize = obstacle.size;
    const verticalLength = obstacleSize * 0.6;
    const horizontalLength = obstacleSize * 0.4;
    const barThickness = obstacleSize * 0.08;
    
    // Calculate goal direction
    const goalDx = goalX - x;
    const goalDy = goalY - y;
    const goalDist = Math.sqrt(goalDx * goalDx + goalDy * goalDy);
    const goalAngle = Math.atan2(goalDy, goalDx);
    
    // Current velocity direction
    const currentSpeed = Math.sqrt(currentVx * currentVx + currentVy * currentVy);
    const currentAngle = currentSpeed > 0.01 ? Math.atan2(currentVy, currentVx) : goalAngle;
    
    // Calculate distance to obstacle
    const obsDx = x - obstacle.x;
    const obsDy = y - obstacle.y;
    const obsDist = Math.sqrt(obsDx * obsDx + obsDy * obsDy);
    
    // Test angles - focus more on goal direction, less extreme angles
    const candidateAngles = [];
    // Reduce angle spread when far from obstacle (don't need extreme avoidance)
    const isCloseToObstacle = obsDist < OBSTACLE_AVOIDANCE_DISTANCE;
    const angleSpread = isCloseToObstacle ? Math.PI * 0.8 : Math.PI * 0.4; // ±40° when far, ±80° when close
    
    // More angles around goal direction (prioritize goal-aligned paths)
    for (let i = -8; i <= 8; i++) {
        candidateAngles.push(goalAngle + (i / 8) * angleSpread);
    }
    
    // Fewer angles around current velocity (for smoothness, but less priority)
    if (currentSpeed > 0.1) {
        for (let i = -3; i <= 3; i++) {
            const testAngle = currentAngle + (i / 3) * angleSpread * 0.3;
            // Only add if not too close to goal angles
            let isUnique = true;
            for (const existing of candidateAngles) {
                if (Math.abs(testAngle - existing) < 0.15) {
                    isUnique = false;
                    break;
                }
            }
            if (isUnique) candidateAngles.push(testAngle);
        }
    }
    
    let bestAngle = goalAngle;
    let bestScore = -Infinity;
    
    // Evaluate each candidate angle
    for (const candidateAngle of candidateAngles) {
        let pathSafe = true;
        let pathClearDistance = 0;
        let minDistanceFromObstacle = Infinity;
        let minDistanceToGoal = goalDist;
        let wouldHitScreenEdge = false;
        
        // Check path ahead in steps
        for (let step = 1; step <= steps; step++) {
            const testDist = (lookaheadDistance / steps) * step;
            const testX = x + Math.cos(candidateAngle) * testDist;
            const testY = y + Math.sin(candidateAngle) * testDist;
            
            // Check if path would hit screen edges (penalize these)
            const distToLeftEdge = testX;
            const distToRightEdge = screenWidth - testX;
            const distToTopEdge = testY;
            const distToBottomEdge = screenHeight - testY;
            const minEdgeDist = Math.min(distToLeftEdge, distToRightEdge, distToTopEdge, distToBottomEdge);
            
            if (minEdgeDist < SCREEN_EDGE_PENALTY_DISTANCE) {
                wouldHitScreenEdge = true;
            }
            
            // Check collision with obstacle
            const wouldCollide = checkTCollision(testX, testY, obstacle.x, obstacle.y,
                obstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
            
            if (wouldCollide) {
                pathSafe = false;
                break;
            }
            
            pathClearDistance = testDist;
            
            // Track minimum distances
            const testGoalDx = goalX - testX;
            const testGoalDy = goalY - testY;
            const testGoalDist = Math.sqrt(testGoalDx * testGoalDx + testGoalDy * testGoalDy);
            minDistanceToGoal = Math.min(minDistanceToGoal, testGoalDist);
            
            const testObsDx = testX - obstacle.x;
            const testObsDy = testY - obstacle.y;
            const testObsDist = Math.sqrt(testObsDx * testObsDx + testObsDy * testObsDy);
            minDistanceFromObstacle = Math.min(minDistanceFromObstacle, testObsDist);
        }
        
        if (pathSafe && pathClearDistance > 0) {
            // Score this path - improved scoring
            const goalAlignment = Math.cos(candidateAngle - goalAngle); // 1 if aligned, -1 if opposite
            const currentAlignment = currentSpeed > 0.1 ? Math.cos(candidateAngle - currentAngle) : 0; // Smoothness
            
            // Obstacle distance score - prefer safe distance but don't require maximum distance
            const safeObstacleDistance = obstacleSize * 0.3; // Safe distance is 30% of obstacle size
            const obstacleDistanceScore = minDistanceFromObstacle > safeObstacleDistance ? 
                1.0 : (minDistanceFromObstacle / safeObstacleDistance); // Full score if safe distance
            
            // Path length score - prefer longer clear paths
            const pathLengthScore = pathClearDistance / lookaheadDistance;
            
            // Goal progress score - prefer paths that get closer to goal
            const goalProgressScore = (goalDist - minDistanceToGoal) / goalDist;
            
            // Screen edge penalty - heavily penalize paths that go to screen edges
            const edgePenalty = wouldHitScreenEdge ? -5 : 0;
            
            // Weighted score - goal alignment is most important, but balanced with other factors
            const score = goalAlignment * 8 + // Goal alignment (most important, increased weight)
                         currentAlignment * 3 + // Smoothness (avoid sharp turns)
                         obstacleDistanceScore * 4 + // Safe distance from obstacle
                         pathLengthScore * 3 + // Path clarity
                         goalProgressScore * 5 + // Progress toward goal
                         edgePenalty; // Penalty for screen edges
            
            if (score > bestScore) {
                bestScore = score;
                bestAngle = candidateAngle;
            }
        }
    }
    
    return { angle: bestAngle, score: bestScore };
};

// Smoothly interpolate between current angle and target angle
const smoothTurn = (currentAngle, targetAngle, maxTurnRate) => {
    // Normalize angles to [-PI, PI]
    let diff = targetAngle - currentAngle;
    while (diff > Math.PI) diff -= 2 * Math.PI;
    while (diff < -Math.PI) diff += 2 * Math.PI;
    
    // Clamp turn rate
    const turnAmount = Math.max(-maxTurnRate, Math.min(maxTurnRate, diff));
    return currentAngle + turnAmount;
};

// T-shaped collision detection with circles
const checkTCollision = (px, py, obsX, obsY, obsRotation, verticalLength, horizontalLength, barThickness, racerRadius) => {
    // Transform point to obstacle's local coordinate system
    const dx = px - obsX;
    const dy = py - obsY;
    const angle = -obsRotation * Math.PI / 180;
    const cos = Math.cos(angle);
    const sin = Math.sin(angle);
    
    // Rotate point to obstacle's local space
    const localX = dx * cos - dy * sin;
    const localY = dx * sin + dy * cos;
    
    // T-shape dimensions
    const halfVertical = verticalLength / 2;
    const halfHorizontal = horizontalLength / 2;
    const halfThickness = barThickness / 2;
    const circleRadius = halfThickness;
    
    // Check collision with expanded bounds (for racer radius)
    const expandedThickness = halfThickness + racerRadius;
    const expandedVertical = halfVertical + racerRadius;
    const expandedHorizontal = halfHorizontal + racerRadius;
    const expandedCircleRadius = circleRadius + racerRadius;
    
    // Check vertical shaft collision
    if (Math.abs(localX) <= expandedThickness && localY >= -expandedVertical && localY <= expandedVertical) {
        return true;
    }
    
    // Check horizontal bar collision
    if (Math.abs(localX) <= expandedHorizontal && localY >= -expandedVertical - expandedThickness && localY <= -expandedVertical + expandedThickness) {
        return true;
    }
    
    // Check circle at top-left (horizontal bar left end)
    const circle1X = -halfHorizontal;
    const circle1Y = -halfVertical;
    const dist1 = Math.sqrt((localX - circle1X) * (localX - circle1X) + (localY - circle1Y) * (localY - circle1Y));
    if (dist1 <= expandedCircleRadius) {
        return true;
    }
    
    // Check circle at top-right (horizontal bar right end)
    const circle2X = halfHorizontal;
    const circle2Y = -halfVertical;
    const dist2 = Math.sqrt((localX - circle2X) * (localX - circle2X) + (localY - circle2Y) * (localY - circle2Y));
    if (dist2 <= expandedCircleRadius) {
        return true;
    }
    
    // Check circle at bottom (vertical shaft bottom end)
    const circle3X = 0;
    const circle3Y = halfVertical;
    const dist3 = Math.sqrt((localX - circle3X) * (localX - circle3X) + (localY - circle3Y) * (localY - circle3Y));
    if (dist3 <= expandedCircleRadius) {
        return true;
    }
    
    return false;
};

const App = () => {
    const canvasRef = useRef(null);
    const animationFrameRef = useRef();
    const gameStateRef = useRef('SETUP');
    const [gameState, setGameState] = useState('SETUP');
    const [dust, setDust] = useState([]);
    const [obstacle, setObstacle] = useState(null);
    const obstacleOrbitAngleRef = useRef(0);
    const [racers, setRacers] = useState([]);
    const [finishedRankings, setFinishedRankings] = useState([]);
    const [bet, setBet] = useState(null);
            const [difficulty, setDifficulty] = useState(4);
    const [statusText, setStatusText] = useState("");
            const [consecutiveWins, setConsecutiveWins] = useState(0);
            const [countdown, setCountdown] = useState(null);
            const [goalReached, setGoalReached] = useState(false);
            const velocityChangeTimersRef = useRef([]);
            const takoImageRef = useRef(null);

    const initializeRace = useCallback((width, height) => {
        setDust(generateDust(width, height));
        setFinishedRankings([]);
        setStatusText("");
        setGoalReached(false);
        setCountdown(3);

        // Generate single giant obstacle in center with flowing movement
        const centerX = width * 0.5;
        const centerY = height * 0.5;
        obstacleOrbitAngleRef.current = 0;
        
        // Random size between min and max
        const obstacleSize = OBSTACLE_SIZE_MIN + Math.random() * (OBSTACLE_SIZE_MAX - OBSTACLE_SIZE_MIN);
        
        // Random rotation direction (clockwise or counterclockwise)
        const rotationDirection = Math.random() > 0.5 ? 1 : -1;
        
        // Random orbit direction
        const orbitDirection = Math.random() > 0.5 ? 1 : -1;
        
        const newObstacle = {
            x: centerX,
            y: centerY,
            centerX: centerX, // Store center for orbital movement
            centerY: centerY,
            rotation: Math.random() * 360, // Random starting rotation
            rotationSpeed: OBSTACLE_ROTATION_SPEED * rotationDirection, // Random direction
            orbitDirection: orbitDirection, // Store orbit direction
            size: obstacleSize // Store the size
        };
        
        setObstacle(newObstacle);

        velocityChangeTimersRef.current.forEach(timer => clearTimeout(timer));
        velocityChangeTimersRef.current = [];

        // Calculate tako (octopus) position and tip (start position for racers)
        // Tako is in bottom left, rotated 45 degrees clockwise, positioned so tentacles are off-screen
        const takoSize = Math.min(width, height) * 0.3; // Size of tako image
        const takoX = takoSize * 0.25; // Further to bottom left
        const takoY = height - takoSize * 0.25; // Further to bottom left
        const takoRotation = 45 * Math.PI / 180; // 45 degrees clockwise
        
        // Calculate tip position (top center of tako head after rotation)
        // The tip is at the top center of the tako before rotation, adjusted for visible head area
        const tipOffsetX = 0;
        const tipOffsetY = -takoSize * 0.3; // Top of visible head area
        const cosRot = Math.cos(takoRotation);
        const sinRot = Math.sin(takoRotation);
        const tipX = takoX + (tipOffsetX * cosRot - tipOffsetY * sinRot);
        const tipY = takoY + (tipOffsetX * sinRot + tipOffsetY * cosRot);

        return HORSES_CONFIG.map((config, index) => {
            // All racers start at the tip of the octopus with velocity 0
            const startX = tipX + (Math.random() - 0.5) * 10; // Small random offset
            const startY = tipY + (Math.random() - 0.5) * 10;
            const vx = 0; // Start with zero velocity
            const vy = 0;

            return {
                ...config,
                x: startX, y: startY, 
                vx, vy,
                angle: Math.atan2(vy, vx),
                tail: [],
                finished: false,
                finalX: 0, finalY: 0,
                stuck: false, // Whether racer is stuck in obstacle
                stuckFrames: 0, // Frames spent stuck
                collisionFrames: 0, // Frames spent colliding
                collisionDirection: null, // Direction of collision (angle in radians)
                escapeBurstFrames: 0, // Frames remaining in escape burst
                prevX: startX, // Previous X position for stuck detection
                prevY: startY, // Previous Y position for stuck detection
                approachingObstacle: false, // Whether racer is approaching obstacle
                avoidingObstacle: false, // Whether racer is actively avoiding obstacle
                avoidanceDirection: null, // Direction to avoid obstacle (angle in radians)
                baseVelocity: BASE_VELOCITY // Store base velocity for re-acceleration
            };
        });
    }, []);

    const checkResultAndSignal = useCallback(async (rankings) => {
        const madeTheCut = rankings.includes(bet);
        
        if (madeTheCut) {
            const rank = rankings.indexOf(bet) + 1;
            setStatusText(`SAFE: Finished #${rank}. No signal sent.`);
            setConsecutiveWins(prev => prev + 1); // Increment consecutive wins
        } else {
            setStatusText('DEFEAT: Transmitting penalty... ⚡');
            setConsecutiveWins(0); // Reset consecutive wins when punishment is transferred
            try {
                // Send the POST request to the ESP32's /lose route.
                // This triggers the handleLose() function on the ESP32 server.
                await fetch(API_LOSE_ENDPOINT, { 
                    method: 'POST' 
                });
            } catch (error) {
                // Handle network issues (e.g., ESP32 is off or IP is wrong)
                console.error("Failed to signal ESP32:", error);
                setStatusText('CONNECTION FAILED to ESP32');
            }
        }
    }, [bet]);

    const gameLoop = useCallback(() => {
        try {
        const canvas = canvasRef.current;
            if (!canvas || gameState !== 'RACING' || countdown !== null) return; // Don't run game loop during countdown

        const W = canvas.width;
        const H = canvas.height;
            const GOAL_RADIUS = Math.max(W, H) * 0.15;

        // Update single giant obstacle: rotation and orbital/flowing movement
        let currentObstacle = obstacle;
        if (obstacle) {
            obstacleOrbitAngleRef.current += OBSTACLE_ORBIT_SPEED * obstacle.orbitDirection;
            
            // Very slow orbital movement around center
            const orbitX = Math.cos(obstacleOrbitAngleRef.current) * OBSTACLE_ORBIT_RADIUS;
            const orbitY = Math.sin(obstacleOrbitAngleRef.current) * OBSTACLE_ORBIT_RADIUS;
            
            // Very slow additional flowing movement (figure-8 pattern) - reduced flow speed
            const flowX = Math.sin(obstacleOrbitAngleRef.current * 2) * OBSTACLE_ORBIT_RADIUS * 0.15;
            const flowY = Math.cos(obstacleOrbitAngleRef.current * 1.5) * OBSTACLE_ORBIT_RADIUS * 0.1;
            
            currentObstacle = {
                ...obstacle,
                x: obstacle.centerX + orbitX + flowX,
                y: obstacle.centerY + orbitY + flowY,
                rotation: (obstacle.rotation + obstacle.rotationSpeed) % 360
            };
            
            setObstacle(currentObstacle);
        }

        setRacers(prevRacers => {
            let currentFinishers = [...finishedRankings];

            const newRacers = prevRacers.map(r => {
                if (r.finished) return r;

                let { x, y, vx, vy, angle, tail, stuck, stuckFrames: prevStuckFrames, collisionDirection, escapeBurstFrames, approachingObstacle, avoidingObstacle, avoidanceDirection, baseVelocity } = r;
                
                // Track previous position to detect if stuck in place
                const prevX = r.prevX !== undefined ? r.prevX : x;
                const prevY = r.prevY !== undefined ? r.prevY : y;
                const positionChange = Math.sqrt((x - prevX) * (x - prevX) + (y - prevY) * (y - prevY));

                // T-shaped collision detection with physical collision
                const racerRadius = Math.max(RACER_WIDTH, RACER_HEIGHT);
                const obstacleSize = currentObstacle ? currentObstacle.size : OBSTACLE_SIZE_MIN;
                const verticalLength = obstacleSize * 0.6;
                const horizontalLength = obstacleSize * 0.4;
                const barThickness = obstacleSize * 0.08;
                
                // ===== STEP 1: Check current collision state =====
                let isColliding = false;
                if (currentObstacle) {
                    isColliding = checkTCollision(x, y, currentObstacle.x, currentObstacle.y, 
                        currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                }
                
                // Track collision frames for immediate escape response
                const collisionFrames = isColliding ? ((r.collisionFrames || 0) + 1) : 0;
                const needsImmediateEscape = collisionFrames >= COLLISION_ESCAPE_FRAMES;
                
                // Detect stuck state - not moving much for several frames
                const currentSpeed = Math.sqrt(vx * vx + vy * vy);
                const isMovingSlowly = currentSpeed < STUCK_VELOCITY_THRESHOLD;
                const isStuckInPlace = positionChange < STUCK_POSITION_THRESHOLD;
                const isStuck = (isColliding || isMovingSlowly) && isStuckInPlace;
                
                // Update stuck frames counter
                const stuckFrames = isStuck ? (prevStuckFrames || 0) + 1 : 0;
                const isTrulyStuck = stuckFrames >= STUCK_DETECTION_FRAMES;
                
                // Determine if we need aggressive escape
                const shouldEscapeAggressively = isTrulyStuck || needsImmediateEscape || (isColliding && isMovingSlowly);
                
                // ===== STEP 2: If currently colliding, immediately push away =====
                if (isColliding && currentObstacle) {
                    // Calculate direction away from obstacle
                    const obsDx = x - currentObstacle.x;
                    const obsDy = y - currentObstacle.y;
                    const obsDist = Math.sqrt(obsDx * obsDx + obsDy * obsDy);
                    
                    if (obsDist > 0.1) {
                        const normalX = obsDx / obsDist;
                        const normalY = obsDy / obsDist;
                        
                        // Find safe position away from obstacle
                        let escapeX = x;
                        let escapeY = y;
                        let foundEscape = false;
                        
                        // Try multiple escape distances and directions
                        const escapeDistances = isTrulyStuck ? [racerRadius * 8, racerRadius * 5, racerRadius * 3] : [racerRadius * 5, racerRadius * 3, racerRadius * 2];
                        
                        for (const escapeDist of escapeDistances) {
                            // Try directly away first
                            escapeX = x + normalX * escapeDist;
                            escapeY = y + normalY * escapeDist;
                            
                            if (!checkTCollision(escapeX, escapeY, currentObstacle.x, currentObstacle.y,
                                currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius)) {
                                foundEscape = true;
                                break;
                            }
                            
                            // Try 8 directions around the normal
                            for (let angleOffset = 0; angleOffset < Math.PI * 2; angleOffset += Math.PI / 4) {
                                const escapeDirX = Math.cos(Math.atan2(normalY, normalX) + angleOffset);
                                const escapeDirY = Math.sin(Math.atan2(normalY, normalX) + angleOffset);
                                escapeX = x + escapeDirX * escapeDist;
                                escapeY = y + escapeDirY * escapeDist;
                                
                                if (!checkTCollision(escapeX, escapeY, currentObstacle.x, currentObstacle.y,
                                    currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius)) {
                                    foundEscape = true;
                                    break;
                                }
                            }
                            
                            if (foundEscape) break;
                        }
                        
                        // If found escape, move there and set velocity away from obstacle
                        if (foundEscape) {
                            const escapeAngle = Math.atan2(escapeY - y, escapeX - x);
                            const escapeSpeed = baseVelocity * (isTrulyStuck ? STUCK_ESCAPE_FORCE : COLLISION_ESCAPE_FORCE);
                            
                            x = escapeX;
                            y = escapeY;
                            vx = Math.cos(escapeAngle) * escapeSpeed;
                            vy = Math.sin(escapeAngle) * escapeSpeed;
                            
                            // Mark that we've escaped this frame
                            isColliding = false;
                        } else {
                            // If no escape found, at least set velocity away from obstacle
                            const escapeAngle = Math.atan2(normalY, normalX);
                            const escapeSpeed = baseVelocity * STUCK_ESCAPE_FORCE;
                            vx = Math.cos(escapeAngle) * escapeSpeed;
                            vy = Math.sin(escapeAngle) * escapeSpeed;
                        }
                    }
                }
                
                // ===== STEP 3: Normal path planning (if not colliding) =====
                if (!isColliding) {
                    const goalX = W;
                    const goalY = 0;
                    const currentAngle = currentSpeed > 0.01 ? Math.atan2(vy, vx) : Math.atan2(goalY - y, goalX - x);
                    
                    // Use path planning
                    const pathPlan = planPath(x, y, goalX, goalY, vx, vy, currentObstacle, racerRadius, 
                        PATH_PLANNING_LOOKAHEAD, PATH_PLANNING_STEPS, W, H);
                    
                    // Smoothly turn towards planned direction
                    const targetAngle = pathPlan.angle;
                    const turnRate = shouldEscapeAggressively ? MAX_TURN_RATE : MIN_TURN_RATE;
                    const smoothedAngle = smoothTurn(currentAngle, targetAngle, turnRate);
                    
                    // Calculate desired velocity magnitude
                    let desiredSpeed = baseVelocity;
                    if (currentObstacle && !isColliding) {
                        const obsDx = x - currentObstacle.x;
                        const obsDy = y - currentObstacle.y;
                        const obsDist = Math.sqrt(obsDx * obsDx + obsDy * obsDy);
                        if (obsDist < OBSTACLE_AVOIDANCE_DISTANCE) {
                            const slowFactor = 0.6 + 0.4 * (obsDist / OBSTACLE_AVOIDANCE_DISTANCE);
                            desiredSpeed = baseVelocity * slowFactor;
                        }
                    }
                    
                    // Calculate target velocity
                    const targetVx = Math.cos(smoothedAngle) * desiredSpeed;
                    const targetVy = Math.sin(smoothedAngle) * desiredSpeed;
                    
                    // Smoothly interpolate velocity
                    const smoothingFactor = VELOCITY_SMOOTHING;
                    vx = vx * (1 - smoothingFactor) + targetVx * smoothingFactor;
                    vy = vy * (1 - smoothingFactor) + targetVy * smoothingFactor;
                    
                    // Add small random variation for natural movement
                    vx += (Math.random() - 0.5) * RANDOM_FACTOR;
                    vy += (Math.random() - 0.5) * RANDOM_FACTOR;
                }
                
                // ===== STEP 4: Check if velocity would cause collision and adjust =====
                if (currentObstacle && !isColliding) {
                    const newX = x + vx;
                    const newY = y + vy;
                    const willCollide = checkTCollision(newX, newY, currentObstacle.x, currentObstacle.y,
                        currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                    
                    if (willCollide) {
                        // Find safe direction that doesn't collide
                        const currentAngle = Math.atan2(vy, vx);
                        let safeAngle = currentAngle;
                        let foundSafe = false;
                        
                        // Try angles around current direction
                        for (let angleOffset = -Math.PI; angleOffset <= Math.PI; angleOffset += Math.PI / 8) {
                            const testAngle = currentAngle + angleOffset;
                            const testVx = Math.cos(testAngle) * baseVelocity;
                            const testVy = Math.sin(testAngle) * baseVelocity;
                            const testX = x + testVx;
                            const testY = y + testVy;
                            
                            if (!checkTCollision(testX, testY, currentObstacle.x, currentObstacle.y,
                                currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius)) {
                                safeAngle = testAngle;
                                foundSafe = true;
                                break;
                            }
                        }
                        
                        if (foundSafe) {
                            // Adjust velocity to safe direction
                            const safeSpeed = Math.sqrt(vx * vx + vy * vy);
                            vx = Math.cos(safeAngle) * safeSpeed;
                            vy = Math.sin(safeAngle) * safeSpeed;
                        } else {
                            // If no safe direction found, move backwards
                            vx = -vx * 0.5;
                            vy = -vy * 0.5;
                        }
                    }
                }
                
                // ===== STEP 5: Apply movement =====
                // Safety checks
                if (!isFinite(vx)) vx = 0;
                if (!isFinite(vy)) vy = 0;
                if (!isFinite(x)) x = 0;
                if (!isFinite(y)) y = 0;
                
                const newX = x + vx;
                const newY = y + vy;
                
                // Final collision check before moving
                if (currentObstacle) {
                    const willCollide = checkTCollision(newX, newY, currentObstacle.x, currentObstacle.y,
                        currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                    
                    if (willCollide) {
                        // Find safe position by moving backwards
                        let safeX = x;
                        let safeY = y;
                        for (let step = 0.1; step <= 1.0; step += 0.1) {
                            const testX = x - vx * step;
                            const testY = y - vy * step;
                            if (!checkTCollision(testX, testY, currentObstacle.x, currentObstacle.y,
                                currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius)) {
                                safeX = testX;
                                safeY = testY;
                                // Update velocity to match actual movement
                                const actualDx = safeX - x;
                                const actualDy = safeY - y;
                                const actualDist = Math.sqrt(actualDx * actualDx + actualDy * actualDy);
                                if (actualDist > 0) {
                                    const actualSpeed = Math.sqrt(vx * vx + vy * vy);
                                    vx = (actualDx / actualDist) * actualSpeed * 0.5;
                                    vy = (actualDy / actualDist) * actualSpeed * 0.5;
                                } else {
                                    vx = 0;
                                    vy = 0;
                                }
                                break;
                            }
                        }
                        x = safeX;
                        y = safeY;
                    } else {
                        x = newX;
                        y = newY;
                    }
                } else {
                    x = newX;
                    y = newY;
                }

                // Safety check: ensure x and y are valid before wall checks
                if (!isFinite(x)) x = 0;
                if (!isFinite(y)) y = 0;
                if (!isFinite(vx)) vx = 0;
                if (!isFinite(vy)) vy = 0;
                
                if (x < 0) { x = 0; vx *= -WALL_BOUNCE_DAMPING; }
                if (x > W) { x = W; vx *= -WALL_BOUNCE_DAMPING; }
                if (y < 0) { y = 0; vy *= -WALL_BOUNCE_DAMPING; }
                if (y > H) { y = H; vy *= -WALL_BOUNCE_DAMPING; }
                
                // Final safety check before calculating angle
                if (!isFinite(vx)) vx = 0;
                if (!isFinite(vy)) vy = 0;
                angle = Math.atan2(vy, vx);
                
                // Safety check on angle
                if (!isFinite(angle)) angle = 0;

                tail = [...tail, { x, y }];
                if (tail.length > TAIL_LENGTH) tail.shift();

                const distToCorner = Math.sqrt(Math.pow(W - x, 2) + Math.pow(0 - y, 2));
                if (distToCorner < GOAL_RADIUS) {
                    setGoalReached(true); // Mark goal as reached (turns yellow)
                    return { ...r, x, y, vx: 0, vy: 0, finished: true, finalX: x, finalY: y };
                }

                return { ...r, x, y, vx, vy, angle, tail, stuck: isTrulyStuck, stuckFrames, collisionFrames, collisionDirection, escapeBurstFrames, prevX: x, prevY: y, approachingObstacle, avoidingObstacle, avoidanceDirection, baseVelocity };
            }).filter(r => {
                // Filter out any racers with invalid positions
                return r && isFinite(r.x) && isFinite(r.y) && isFinite(r.vx) && isFinite(r.vy);
            });

            newRacers.forEach(r => {
                if (r && r.finished && !currentFinishers.includes(r.id)) {
                    currentFinishers.push(r.id);
                }
            });

            if (currentFinishers.length !== finishedRankings.length) {
                setFinishedRankings(currentFinishers);
            }

            if (currentFinishers.length >= difficulty) {
                gameStateRef.current = 'FINISHED';
                setGameState('FINISHED');
            }

            return newRacers;
        });

        if (gameState === 'RACING') {
            animationFrameRef.current = requestAnimationFrame(gameLoop);
        }
        } catch (error) {
            console.error('Error in gameLoop:', error);
            // Reset racers to safe state if error occurs
            setRacers(prevRacers => {
                if (!prevRacers || prevRacers.length === 0) return prevRacers;
                return prevRacers.map(r => ({
                    ...r,
                    vx: (r && isFinite(r.vx)) ? r.vx : BASE_VELOCITY * 0.5,
                    vy: (r && isFinite(r.vy)) ? r.vy : BASE_VELOCITY * 0.5,
                    x: (r && isFinite(r.x)) ? r.x : 0,
                    y: (r && isFinite(r.y)) ? r.y : 0
                }));
            });
        }
    }, [gameState, finishedRankings, difficulty, obstacle, countdown]);

    const draw = useCallback(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const W = canvas.width; const H = canvas.height;

        ctx.fillStyle = TRACK_COLOR;
        ctx.fillRect(0, 0, W, H);
        dust.forEach(d => {
            ctx.fillStyle = `rgba(0,0,0,${d.opacity})`;
            ctx.beginPath(); ctx.arc(d.x, d.y, d.size, 0, Math.PI * 2); ctx.fill();
        });

        const GOAL_RADIUS = Math.max(W, H) * 0.15;
        ctx.beginPath(); ctx.moveTo(W, 0);
        ctx.arc(W, 0, GOAL_RADIUS, Math.PI / 2, Math.PI, false);
        ctx.closePath();
        // Goal turns yellow (egg yolk color) when reached
        if (goalReached) {
            ctx.fillStyle = 'rgba(255, 220, 100, 0.4)'; // Egg yolk yellow
            ctx.strokeStyle = 'rgba(255, 200, 50, 0.8)';
        } else {
            ctx.fillStyle = 'rgba(255, 255, 255, 0.05)';
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
        }
        ctx.fill();
        ctx.lineWidth = 2; ctx.stroke();
        
        // Draw tako (octopus) image in bottom left, rotated 45 degrees clockwise
        // Positioned so tentacles are naturally off-screen
        if (takoImageRef.current && takoImageRef.current.complete) {
            const takoSize = Math.min(W, H) * 0.3;
            const takoX = takoSize * 0.25; // Further to bottom left
            const takoY = H - takoSize * 0.25; // Further to bottom left
            
            ctx.save();
            ctx.translate(takoX, takoY);
            ctx.rotate(45 * Math.PI / 180); // 45 degrees clockwise
            
            // No clipping needed - tentacles are naturally off-screen due to position
            ctx.drawImage(takoImageRef.current, -takoSize * 0.5, -takoSize * 0.5, takoSize, takoSize);
            ctx.restore();
        }
        
        if (obstacle) {
            ctx.save();
            ctx.translate(obstacle.x, obstacle.y);
            ctx.rotate((obstacle.rotation * Math.PI) / 180);
            
            ctx.strokeStyle = '#888888';
            ctx.fillStyle = '#888888';
            ctx.lineWidth = 6;
            ctx.lineCap = 'round';
            
            // Draw T-shape (IUD-like): vertical line with horizontal bar at top
            const obstacleSize = obstacle.size || OBSTACLE_SIZE_MIN;
            const verticalLength = obstacleSize * 0.6; // Main vertical shaft
            const horizontalLength = obstacleSize * 0.4; // Top horizontal bar
            const barThickness = obstacleSize * 0.08;
            
            // Vertical shaft
            ctx.beginPath();
            ctx.moveTo(0, -verticalLength/2);
            ctx.lineTo(0, verticalLength/2);
            ctx.stroke();
            
            // Horizontal bar at top
            ctx.beginPath();
            ctx.moveTo(-horizontalLength/2, -verticalLength/2);
            ctx.lineTo(horizontalLength/2, -verticalLength/2);
            ctx.stroke();
            
            // Add small rounded ends for IUD-like appearance
            ctx.beginPath();
            ctx.arc(-horizontalLength/2, -verticalLength/2, barThickness/2, 0, Math.PI * 2);
            ctx.fill();
            ctx.beginPath();
            ctx.arc(horizontalLength/2, -verticalLength/2, barThickness/2, 0, Math.PI * 2);
            ctx.fill();
            ctx.beginPath();
            ctx.arc(0, verticalLength/2, barThickness/2, 0, Math.PI * 2);
            ctx.fill();
            
            ctx.shadowColor = 'rgba(0, 0, 0, 0.3)';
            ctx.shadowBlur = 5;
            ctx.shadowOffsetX = 2;
            ctx.shadowOffsetY = 2;
            
            ctx.restore();
        }
        
        ctx.shadowColor = 'transparent';
        ctx.shadowBlur = 0;
        ctx.shadowOffsetX = 0;
        ctx.shadowOffsetY = 0;
        
        racers.forEach(r => {
            ctx.beginPath();
            for (let i = 0; i < r.tail.length; i++) {
                const opacity = r.finished ? 0.05 : (i / TAIL_LENGTH) * 0.3;
                ctx.strokeStyle = `rgba(255, 255, 255, ${opacity})`;
                ctx.lineWidth = RACER_SIZE * 0.5;
                i === 0 ? ctx.moveTo(r.tail[i].x, r.tail[i].y) : ctx.lineTo(r.tail[i].x, r.tail[i].y);
            }
            ctx.stroke();

            const isMyBet = r.id === bet;
            ctx.fillStyle = r.finished ? r.color : '#ffffff';
            ctx.save();
            ctx.translate(r.x, r.y);
            ctx.rotate(r.angle);
            ctx.beginPath();
            const currentRacerWidth = r.finished ? RACER_WIDTH * 1.5 : RACER_WIDTH;
            const currentRacerHeight = r.finished ? RACER_HEIGHT * 1.5 : RACER_HEIGHT;
            ctx.ellipse(0, 0, currentRacerWidth, currentRacerHeight, 0, 0, Math.PI * 2);
            ctx.fill();
            ctx.restore();
            
            if (isMyBet && !r.finished) {
                ctx.save();
                ctx.font = 'bold 12px system-ui';
                ctx.fillStyle = '#ffffff';
                ctx.strokeStyle = '#000000';
                ctx.lineWidth = 3;
                ctx.textAlign = 'center';
                ctx.textBaseline = 'bottom';
                ctx.strokeText('YOU', r.x, r.y - 10);
                ctx.fillText('YOU', r.x, r.y - 10);
                ctx.restore();
            }
        });
    }, [racers, dust, obstacle, bet, goalReached]);

    // Load tako (octopus) image
    useEffect(() => {
        const img = new Image();
        img.src = '/tako.png';
        img.onload = () => {
            takoImageRef.current = img;
        };
        img.onerror = () => {
            console.warn('Tako image not found. Please place tako.png in the public folder.');
        };
    }, []);

    // Countdown timer
    useEffect(() => {
        if (countdown !== null && countdown > 0 && gameState === 'RACING') {
            const timer = setTimeout(() => {
                setCountdown(countdown - 1);
            }, 1000);
            return () => clearTimeout(timer);
        } else if (countdown === 0) {
            // Countdown finished - start the race
            setCountdown(null);
            // Give racers initial velocity based on their target velocities (maintains path variation)
            setRacers(prevRacers => prevRacers.map(r => ({
                ...r,
                vx: (Math.random() * BASE_VELOCITY) + BASE_VELOCITY/2,
                vy: (-(Math.random() * BASE_VELOCITY) - BASE_VELOCITY/2)
            })));
        }
    }, [countdown, gameState]);

    useEffect(() => {
        const resizeCanvas = () => {
            if (canvasRef.current) {
                canvasRef.current.width = window.innerWidth;
                canvasRef.current.height = window.innerHeight;
                if (gameState === 'SETUP') setRacers(initializeRace(window.innerWidth, window.innerHeight));
                setDust(generateDust(window.innerWidth, window.innerHeight));
            }
        };
        resizeCanvas();
        window.addEventListener('resize', resizeCanvas);
        return () => window.removeEventListener('resize', resizeCanvas);
    }, [initializeRace, gameState]);

    useEffect(() => {
        if (gameState !== 'RACING' || racers.length === 0) {
            velocityChangeTimersRef.current.forEach(timer => clearTimeout(timer));
            velocityChangeTimersRef.current = [];
            return;
        }

        if (velocityChangeTimersRef.current.length === 0) {
            racers.forEach((racer) => {
                const scheduleNextChange = (racerId) => {
                    // Random interval between 0.2 and 0.6 seconds (200-600ms)
                    const delay = Math.random() * 400 + 200; 
                    
                    const timer = setTimeout(() => {
                        setRacers(prevRacers => {
                            const currentRacer = prevRacers.find(r => r.id === racerId);
                            if (!currentRacer || currentRacer.finished) {
                                return prevRacers;
                            }
                            
                            // Vary base velocity slightly for more natural movement variation
                            // Path planning will use this to adjust speed
                            const velocityVariation = 0.7 + Math.random() * 0.6; // 70% to 130% of base
                            const newBaseVelocity = BASE_VELOCITY * velocityVariation;
                            
                            return prevRacers.map(r => 
                                r.id === racerId ? { ...r, baseVelocity: newBaseVelocity } : r
                            );
                        });
                        
                        if (gameStateRef.current === 'RACING') {
                            scheduleNextChange(racerId);
                        }
                    }, delay);
                    
                    velocityChangeTimersRef.current.push(timer);
                };
                
                scheduleNextChange(racer.id);
            });
        }

        return () => {
            velocityChangeTimersRef.current.forEach(timer => clearTimeout(timer));
            velocityChangeTimersRef.current = [];
        };
    }, [gameState, racers.length]);

    useEffect(() => {
        if (gameState === 'RACING') {
            animationFrameRef.current = requestAnimationFrame(gameLoop);
        } else if (gameState === 'FINISHED') {
            cancelAnimationFrame(animationFrameRef.current);
            checkResultAndSignal(finishedRankings);
        }
        return () => cancelAnimationFrame(animationFrameRef.current);
    }, [gameState, gameLoop, checkResultAndSignal, finishedRankings]);

    useEffect(() => {
        let frameId;
        const renderLoop = () => {
        draw();
            if (gameState === 'RACING' || gameState === 'FINISHED') {
                frameId = requestAnimationFrame(renderLoop);
            }
        };

        renderLoop();
        return () => cancelAnimationFrame(frameId);
    }, [draw, gameState]);

    const RacerOutline = ({ racer }) => {
        if (!canvasRef.current) return null;
        
        return (
            <div 
            style={{
                    position: 'absolute',
                    left: `${racer.x}px`,
                    top: `${racer.y}px`,
                    transform: 'translate(-50%, -50%)',
                    width: '24px',
                    height: '24px',
                border: `2px solid ${racer.color}`,
                borderRadius: '4px',
                    opacity: (gameState === 'RACING' && !racer.finished) ? 0.8 : 0,
                    boxShadow: `0 0 8px ${racer.color}80`,
                    pointerEvents: 'none',
                    transition: 'opacity 0.3s'
            }}
        />
    );
    };

    return (
        <div style={{ position: 'fixed', top: 0, left: 0, right: 0, bottom: 0, backgroundColor: '#000000', overflow: 'hidden', fontFamily: 'system-ui, sans-serif' }}>
            <div style={{ position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, filter: 'contrast(1.2) brightness(1.1) blur(0.2px) saturate(0.9)', zIndex: 1 }}>
                <canvas ref={canvasRef} style={{ display: 'block', width: '100%', height: '100%' }} />
            </div>

            {gameState === 'RACING' && (
                <>
                    <div style={{ position: 'fixed', top: 0, left: 0, right: 0, bottom: 0, zIndex: 5000, pointerEvents: 'none' }}>
                {racers.map(r => <RacerOutline key={r.id} racer={r} />)}
            </div>
                    {/* Countdown display */}
                    {countdown !== null && countdown > 0 && (
                        <div style={{ 
                            position: 'fixed', 
                            top: '50%', 
                            left: '50%', 
                            transform: 'translate(-50%, -50%)',
                            zIndex: 20000, 
                            pointerEvents: 'none',
                            textAlign: 'center'
                        }}>
                            <div style={{ 
                                fontSize: '120px', 
                                fontWeight: '900', 
                                color: '#ffffff',
                                textShadow: '0 0 40px rgba(255, 255, 255, 0.8), 0 0 80px rgba(255, 255, 255, 0.5)',
                                lineHeight: '1',
                                animation: 'pulse 0.5s ease-in-out'
                            }}>
                                {countdown}
                            </div>
                        </div>
                    )}
                    {/* Win streak display during race */}
                    {consecutiveWins > 0 && countdown === null && (
                        <div style={{ 
                            position: 'fixed', 
                            top: '20px', 
                            right: '20px', 
                            zIndex: 10001, 
                            pointerEvents: 'none',
                            backgroundColor: 'rgba(16, 185, 129, 0.15)',
                            backdropFilter: 'blur(8px)',
                            border: '1px solid rgba(16, 185, 129, 0.3)',
                            borderRadius: '12px',
                            padding: '12px 20px',
                            display: 'flex',
                            alignItems: 'center',
                            gap: '10px',
                            boxShadow: '0 4px 12px rgba(0, 0, 0, 0.3)'
                        }}>
                            <span style={{ fontSize: '20px' }}>🔥</span>
                            <div>
                                <div style={{ fontSize: '11px', color: '#6ee7b7', textTransform: 'uppercase', letterSpacing: '0.1em', fontWeight: 'bold' }}>Win Streak</div>
                                <div style={{ fontSize: '24px', color: '#10b981', fontWeight: '900', lineHeight: '1' }}>{consecutiveWins}</div>
                            </div>
                        </div>
                    )}
                </>
            )}

            <div style={{ position: 'fixed', top: 0, left: 0, right: 0, bottom: 0, zIndex: 10000, pointerEvents: 'none', display: 'flex', alignItems: 'center', justifyContent: 'center', padding: '20px' }}>
                {gameState === 'SETUP' && (
                    <div style={{ pointerEvents: 'auto', backgroundColor: 'rgba(17, 24, 39, 0.98)', backdropFilter: 'blur(16px)', borderRadius: '28px', border: '1px solid rgba(255, 255, 255, 0.12)', boxShadow: '0 25px 50px -12px rgba(0, 0, 0, 0.6), 0 0 0 1px rgba(255, 255, 255, 0.05)', padding: '40px', maxWidth: '650px', width: '100%' }}>
                        <div style={{ textAlign: 'center', marginBottom: '8px' }}>
                            <h1 style={{ fontSize: '42px', fontWeight: '900', background: 'linear-gradient(135deg, #ffffff 0%, #a0aec0 100%)', WebkitBackgroundClip: 'text', WebkitTextFillColor: 'transparent', backgroundClip: 'text', marginBottom: '8px', letterSpacing: '-0.03em' }}>TASER DERBY</h1>
                            <p style={{ fontSize: '13px', color: '#9ca3af', letterSpacing: '0.05em', textTransform: 'uppercase' }}>Digital stakes, physical consequence.</p>
                        </div>

                        {/* Win streak display in setup */}
                        {consecutiveWins > 0 && (
                            <div style={{ 
                                backgroundColor: 'rgba(16, 185, 129, 0.15)', 
                                border: '1px solid rgba(16, 185, 129, 0.3)',
                                borderRadius: '12px', 
                                padding: '16px', 
                                marginBottom: '32px',
                                display: 'flex',
                                alignItems: 'center',
                                justifyContent: 'center',
                                gap: '12px'
                            }}>
                                <span style={{ fontSize: '24px' }}>🔥</span>
                                <div style={{ textAlign: 'center' }}>
                                    <div style={{ fontSize: '11px', color: '#6ee7b7', textTransform: 'uppercase', letterSpacing: '0.1em', fontWeight: 'bold', marginBottom: '4px' }}>Current Win Streak</div>
                                    <div style={{ fontSize: '32px', color: '#10b981', fontWeight: '900', lineHeight: '1' }}>{consecutiveWins}</div>
                            </div>
                            </div>
                        )}
                        
                        <div style={{ backgroundColor: 'rgba(0, 0, 0, 0.5)', padding: '28px', borderRadius: '20px', marginBottom: '32px', border: '1px solid rgba(255, 255, 255, 0.08)' }}>
                            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '20px', color: '#d1d5db', fontSize: '13px', fontFamily: 'monospace', textTransform: 'uppercase', letterSpacing: '0.1em' }}>
                                <span style={{ fontWeight: '600' }}>Risk Level</span>
                                <span style={{ color: '#818cf8', fontWeight: 'bold', fontSize: '14px' }}>TOP {difficulty} Finishers</span>
                            </div>
                            <input 
                                type="range" 
                                min="1" 
                                max="8" 
                                step="1" 
                                value={difficulty} 
                                onChange={(e) => setDifficulty(Number(e.target.value))}
                                style={{ 
                                    width: '100%', 
                                    height: '10px', 
                                    borderRadius: '5px', 
                                    background: 'linear-gradient(to right, #4f46e5 0%, #818cf8 50%, #a5b4fc 100%)',
                                    outline: 'none', 
                                    cursor: 'pointer', 
                                    WebkitAppearance: 'none', 
                                    appearance: 'none'
                                }} 
                            />
                            <div style={{ display: 'flex', justifyContent: 'space-between', marginTop: '8px', fontSize: '11px', color: '#6b7280' }}>
                                <span>Easier</span>
                                <span>Harder</span>
                            </div>
                            <p style={{ textAlign: 'center', color: '#9ca3af', fontSize: '13px', marginTop: '16px', marginBottom: 0, lineHeight: '1.5' }}>
                                Race ends when <strong style={{ color: '#d1d5db' }}>{difficulty}</strong> racer{difficulty !== 1 ? 's' : ''} reach the goal. You must be one of them.
                            </p>
                        </div>

                        <div style={{ marginBottom: '24px' }}>
                            <div style={{ fontSize: '12px', color: '#9ca3af', textTransform: 'uppercase', letterSpacing: '0.1em', marginBottom: '16px', textAlign: 'center', fontWeight: '600' }}>Select Your Racer</div>
                            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '12px' }}>
                            {HORSES_CONFIG.map(h => (
                                    <button 
                                        key={h.id} 
                                        onClick={() => { setBet(h.id); setRacers(initializeRace(window.innerWidth, window.innerHeight)); gameStateRef.current = 'RACING'; setGameState('RACING'); }} 
                                        onMouseEnter={(e) => { 
                                            e.currentTarget.style.transform = 'scale(1.08) translateY(-2px)'; 
                                            e.currentTarget.style.backgroundColor = 'rgba(55, 65, 81, 0.8)'; 
                                            e.currentTarget.style.boxShadow = `0 8px 16px rgba(0, 0, 0, 0.4), 0 0 0 2px ${h.color}40`;
                                        }} 
                                        onMouseLeave={(e) => { 
                                            e.currentTarget.style.transform = 'scale(1) translateY(0)'; 
                                            e.currentTarget.style.backgroundColor = 'rgba(31, 41, 55, 0.6)'; 
                                            e.currentTarget.style.boxShadow = bet === h.id ? `0 4px 12px rgba(0, 0, 0, 0.3), 0 0 0 2px ${h.color}` : '0 4px 12px rgba(0, 0, 0, 0.3)';
                                        }} 
                                        style={{ 
                                            padding: '20px 12px', 
                                            borderRadius: '16px', 
                                            border: `2px solid ${bet === h.id ? h.color : 'rgba(255, 255, 255, 0.1)'}`, 
                                            backgroundColor: bet === h.id ? 'rgba(31, 41, 55, 0.8)' : 'rgba(31, 41, 55, 0.6)', 
                                            cursor: 'pointer', 
                                            transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)', 
                                            display: 'flex', 
                                            flexDirection: 'column', 
                                            alignItems: 'center', 
                                            justifyContent: 'center', 
                                            position: 'relative', 
                                            overflow: 'hidden',
                                            boxShadow: bet === h.id ? `0 4px 12px rgba(0, 0, 0, 0.3), 0 0 0 2px ${h.color}` : '0 4px 12px rgba(0, 0, 0, 0.3)'
                                        }}
                                    >
                                        <span style={{ fontSize: '40px', marginBottom: '8px', filter: bet === h.id ? 'grayscale(0) brightness(1.1)' : 'grayscale(0.3)', transition: 'filter 0.3s' }}>{h.emoji}</span>
                                        <div style={{ fontSize: '10px', color: '#9ca3af', fontWeight: '600', textAlign: 'center' }}>{h.name}</div>
                                        {bet === h.id && (
                                            <div style={{ 
                                                position: 'absolute', 
                                                bottom: 0, 
                                                left: 0, 
                                                right: 0, 
                                                height: '3px', 
                                                background: `linear-gradient(90deg, transparent, ${h.color}, transparent)`,
                                                animation: 'pulse 2s ease-in-out infinite'
                                            }} />
                                        )}
                                </button>
                            ))}
                            </div>
                        </div>
                    </div>
                )}

                {gameState === 'FINISHED' && (
                    <div style={{ pointerEvents: 'auto', backgroundColor: 'rgba(17, 24, 39, 0.98)', backdropFilter: 'blur(16px)', borderRadius: '28px', border: '1px solid rgba(255, 255, 255, 0.12)', boxShadow: '0 25px 50px -12px rgba(0, 0, 0, 0.6), 0 0 0 1px rgba(255, 255, 255, 0.05)', padding: '40px', maxWidth: '550px', width: '100%', textAlign: 'center' }}>
                        <div style={{ marginBottom: '32px' }}>
                            <h2 style={{ fontSize: '32px', fontWeight: '900', color: statusText.includes('DEFEAT') ? '#ffffff' : '#10b981', textShadow: statusText.includes('DEFEAT') ? '0 0 20px rgba(220, 38, 38, 0.8), 0 2px 4px rgba(0, 0, 0, 0.5)' : '0 0 20px rgba(16, 185, 129, 0.5), 0 2px 4px rgba(0, 0, 0, 0.5)', marginBottom: '12px', letterSpacing: '-0.02em' }}>
                                {statusText.includes('DEFEAT') ? 'INSEMINATION FAILED' : 'INSEMINATION SUCCESS'}
                            </h2>
                            <div style={{ fontSize: '16px', fontWeight: 'bold', color: statusText.includes('DEFEAT') ? '#ffffff' : '#10b981', padding: '12px 20px', backgroundColor: statusText.includes('DEFEAT') ? 'rgba(220, 38, 38, 0.9)' : 'rgba(16, 185, 129, 0.1)', borderRadius: '12px', border: `1px solid ${statusText.includes('DEFEAT') ? 'rgba(220, 38, 38, 1)' : 'rgba(16, 185, 129, 0.3)'}`, display: 'inline-block' }}>
                                {statusText}
                            </div>
                        </div>

                        {/* Win streak display in results */}
                        {consecutiveWins > 0 && (
                            <div style={{ 
                                backgroundColor: 'rgba(16, 185, 129, 0.15)', 
                                border: '1px solid rgba(16, 185, 129, 0.3)',
                                borderRadius: '16px', 
                                padding: '20px', 
                                marginBottom: '32px',
                                display: 'flex',
                                alignItems: 'center',
                                justifyContent: 'center',
                                gap: '16px'
                            }}>
                                <span style={{ fontSize: '32px' }}>🔥</span>
                                <div style={{ textAlign: 'center' }}>
                                    <div style={{ fontSize: '12px', color: '#6ee7b7', textTransform: 'uppercase', letterSpacing: '0.1em', fontWeight: 'bold', marginBottom: '6px' }}>Current Win Streak</div>
                                    <div style={{ fontSize: '40px', color: '#10b981', fontWeight: '900', lineHeight: '1' }}>{consecutiveWins}</div>
                                </div>
                            </div>
                        )}
                        
                        <div style={{ marginBottom: '32px', textAlign: 'left', backgroundColor: 'rgba(0, 0, 0, 0.4)', padding: '20px', borderRadius: '16px', border: '1px solid rgba(255, 255, 255, 0.08)', maxHeight: '280px', overflowY: 'auto' }}>
                            <h3 style={{ fontSize: '11px', fontWeight: 'bold', color: '#9ca3af', marginBottom: '16px', textTransform: 'uppercase', letterSpacing: '0.1em' }}>Qualified Specimens (Top {difficulty})</h3>
                            <ol style={{ listStyle: 'none', padding: 0, margin: 0 }}>
                                {finishedRankings.map((rId, idx) => {
                                    const racer = HORSES_CONFIG.find(h => h.id === rId);
                                    const isMyBet = rId === bet;
                                    return (
                                        <li key={rId} style={{ 
                                            display: 'flex', 
                                            alignItems: 'center', 
                                            padding: '12px', 
                                            borderRadius: '12px', 
                                            marginBottom: '8px', 
                                            backgroundColor: isMyBet ? 'rgba(99, 102, 241, 0.2)' : 'rgba(255, 255, 255, 0.03)', 
                                            border: isMyBet ? `1px solid ${racer.color}60` : '1px solid rgba(255, 255, 255, 0.05)',
                                            transition: 'all 0.2s'
                                        }}>
                                            <span style={{ color: '#6b7280', fontFamily: 'monospace', width: '28px', fontSize: '14px', fontWeight: 'bold' }}>{idx + 1}.</span>
                                            <span style={{ marginRight: '12px', fontSize: '24px' }}>{racer.emoji}</span>
                                            <span style={{ color: isMyBet ? '#ffffff' : '#d1d5db', fontWeight: isMyBet ? 'bold' : '600', flex: 1, fontSize: '14px' }}>{racer.name}</span>
                                            {isMyBet && (
                                                <span style={{ 
                                                    fontSize: '11px', 
                                                    backgroundColor: racer.color, 
                                                    color: '#ffffff', 
                                                    padding: '6px 12px', 
                                                    borderRadius: '12px', 
                                                    marginLeft: 'auto',
                                                    fontWeight: 'bold',
                                                    textTransform: 'uppercase',
                                                    letterSpacing: '0.05em'
                                                }}>YOU</span>
                                            )}
                                        </li>
                                    );
                                })}
                            </ol>
                        </div>

                        <button 
                            onClick={() => { gameStateRef.current = 'SETUP'; setGameState('SETUP'); }} 
                            onMouseEnter={(e) => {
                                e.currentTarget.style.backgroundColor = '#f3f4f6';
                                e.currentTarget.style.transform = 'translateY(-2px)';
                                e.currentTarget.style.boxShadow = '0 8px 16px rgba(0, 0, 0, 0.3)';
                            }} 
                            onMouseLeave={(e) => {
                                e.currentTarget.style.backgroundColor = '#ffffff';
                                e.currentTarget.style.transform = 'translateY(0)';
                                e.currentTarget.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.2)';
                            }} 
                            style={{ 
                                width: '100%', 
                                backgroundColor: '#ffffff', 
                                color: '#000000', 
                                padding: '18px 32px', 
                                borderRadius: '16px', 
                                fontWeight: 'bold', 
                                fontSize: '15px', 
                                letterSpacing: '0.05em', 
                                border: 'none', 
                                cursor: 'pointer', 
                                transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
                                boxShadow: '0 4px 12px rgba(0, 0, 0, 0.2)',
                                textTransform: 'uppercase'
                            }}
                        >
                            Run New Test
                        </button>
                    </div>
                )}
            </div>
        </div>
    );
};

export default App;