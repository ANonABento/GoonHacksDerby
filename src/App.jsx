import React, { useState, useEffect, useRef, useCallback } from 'react';

// --- CONFIGURATION CONSTANTS ---
const ESP32_IP_ADDRESS = '192.168.4.1'; 
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
const OBSTACLE_ROTATION_SPEED = 0.02; // Even slower rotation (degrees per frame)
const OBSTACLE_ORBIT_RADIUS = 30; // Smaller radius for flowing/orbital movement
const OBSTACLE_ORBIT_SPEED = 0.003; // Very slow orbital movement
const BASE_VELOCITY = 0.8;
const MIN_EXPECTED_VELOCITY = BASE_VELOCITY * 0.3; // Minimum velocity racers should maintain (30% of base)
const STUCK_VELOCITY_THRESHOLD = MIN_EXPECTED_VELOCITY; // Use minimum expected velocity as threshold
const ESCAPE_FORCE = 2.0; // Force to apply when escaping
const STUCK_DURATION = 10; // Frames to stay stuck before intelligent escape (reduced)
const WIGGLE_STRENGTH = 0.3; // Strength of wiggling movement
const ESCAPE_BURST_VELOCITY = 3.0; // High velocity burst for intelligent escape
const ESCAPE_BURST_DURATION = 15; // Frames to maintain escape burst
const OBSTACLE_AVOIDANCE_DISTANCE = 250; // Distance to start detecting and avoiding obstacle
const OBSTACLE_DECELERATION_DISTANCE = 200; // Distance to start decelerating
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

            // Path variation: alternate between "up then right" and "right then up" strategies
            // Half go up-first, half go right-first
            const goRightFirst = index % 2 === 0; // Alternate racers
            const rightBias = goRightFirst ? 0.7 : 0.3; // Stronger horizontal component for right-first
            const upBias = goRightFirst ? 0.3 : 0.7; // Stronger vertical component for up-first
            
            const targetVx = BASE_VELOCITY * (0.5 + Math.random() * 0.5) * rightBias;
            const targetVy = -BASE_VELOCITY * (0.5 + Math.random() * 0.5) * upBias;

            return {
                ...config,
                x: startX, y: startY, 
                vx, vy,
                targetVx, targetVy, // Varied target velocity for path diversity
                angle: Math.atan2(vy, vx),
                tail: [],
                finished: false,
                finalX: 0, finalY: 0,
                stuck: false, // Whether racer is stuck in obstacle
                stuckFrames: 0, // Frames spent stuck
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
            setStatusText('DEFEAT: Did not make the cut. Transmitting penalty... ⚡');
            setConsecutiveWins(0); // Reset consecutive wins when punishment is transferred
            try {
                if (ESP32_IP_ADDRESS !== '192.168.4.1') {
                    fetch(API_LOSE_ENDPOINT, { method: 'POST' }).catch(e => console.warn("Signal dispatched quietly"));
                } else {
                    console.warn("Skipping ESP32 signal: Default IP in use.");
                }
            } catch (error) {
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
            
            // Very slow additional flowing movement (figure-8 pattern)
            const flowX = Math.sin(obstacleOrbitAngleRef.current * 2) * OBSTACLE_ORBIT_RADIUS * 0.3;
            const flowY = Math.cos(obstacleOrbitAngleRef.current * 1.5) * OBSTACLE_ORBIT_RADIUS * 0.2;
            
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

                let { x, y, vx, vy, targetVx, targetVy, angle, tail, stuck, stuckFrames, collisionDirection, escapeBurstFrames, approachingObstacle, avoidingObstacle, avoidanceDirection, baseVelocity } = r;
                
                // Track previous position to detect if stuck in place
                const prevX = r.prevX !== undefined ? r.prevX : x;
                const prevY = r.prevY !== undefined ? r.prevY : y;

                // T-shaped collision detection with physical collision
                const racerRadius = Math.max(RACER_WIDTH, RACER_HEIGHT);
                const obstacleSize = currentObstacle ? currentObstacle.size : OBSTACLE_SIZE_MIN;
                const verticalLength = obstacleSize * 0.6;
                const horizontalLength = obstacleSize * 0.4;
                const barThickness = obstacleSize * 0.08;
                
                // Check if currently colliding (needed for avoidance system)
                let isColliding = false;
                if (currentObstacle) {
                    isColliding = checkTCollision(x, y, currentObstacle.x, currentObstacle.y, 
                        currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                }
                
                // Enhanced obstacle avoidance - smart scanning and avoidance system
                // Always active when near obstacle or colliding
                if (currentObstacle && escapeBurstFrames === 0) {
                    const obsDx = x - currentObstacle.x;
                    const obsDy = y - currentObstacle.y;
                    const obsDist = Math.sqrt(obsDx*obsDx + obsDy*obsDy);
                    const currentSpeed = Math.sqrt(vx * vx + vy * vy);
                    
                    // Check if within detection range or currently colliding
                    // Always avoid if colliding, or if close and heading towards obstacle
                    const futureX = x + vx * 20;
                    const futureY = y + vy * 20;
                    const wouldHit = checkTCollision(futureX, futureY, currentObstacle.x, currentObstacle.y,
                        currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                    
                    const shouldAvoid = isColliding || (obsDist < OBSTACLE_AVOIDANCE_DISTANCE && (wouldHit || obsDist < racerRadius * 5));
                    
                    if (shouldAvoid) {
                        approachingObstacle = true;
                        avoidingObstacle = true;
                        
                        // Calculate best avoidance direction with comprehensive scanning
                        const goalDx = W - x;
                        const goalDy = 0 - y;
                        const goalAngle = Math.atan2(goalDy, goalDx);
                        
                        // Scan multiple directions (16 directions for better coverage)
                        const avoidanceCandidates = [];
                        const scanDirections = 16;
                        
                        for (let i = 0; i < scanDirections; i++) {
                            const scanAngle = (Math.PI * 2 * i) / scanDirections;
                            avoidanceCandidates.push({ angle: scanAngle, score: 0 });
                        }
                        
                        // Score each candidate direction
                        for (let i = 0; i < avoidanceCandidates.length; i++) {
                            const candidate = avoidanceCandidates[i];
                            
                            // Test multiple points along this direction to ensure path is safe
                            let pathSafe = true;
                            let maxSafeDistance = 0;
                            
                            for (let step = 1; step <= 5; step++) {
                                const testDistance = racerRadius * 3 * step;
                                const testX = x + Math.cos(candidate.angle) * testDistance;
                                const testY = y + Math.sin(candidate.angle) * testDistance;
                                
                                const isSafe = !checkTCollision(testX, testY, currentObstacle.x, currentObstacle.y,
                                    currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                                
                                if (isSafe) {
                                    maxSafeDistance = testDistance;
                                } else {
                                    if (step === 1) {
                                        pathSafe = false; // Immediate collision
                                    }
                                    break;
                                }
                            }
                            
                            if (pathSafe && maxSafeDistance > 0) {
                                // Calculate scores
                                const testX = x + Math.cos(candidate.angle) * maxSafeDistance;
                                const testY = y + Math.sin(candidate.angle) * maxSafeDistance;
                                const testObsDx = testX - currentObstacle.x;
                                const testObsDy = testY - currentObstacle.y;
                                const testObsDist = Math.sqrt(testObsDx*testObsDx + testObsDy*testObsDy);
                                
                                const distanceScore = testObsDist * 3; // Strong preference for moving away
                                const goalAlignment = Math.cos(candidate.angle - goalAngle);
                                const goalScore = goalAlignment * 5; // Moderate goal preference
                                const pathLengthScore = maxSafeDistance * 0.5; // Prefer longer safe paths
                                
                                candidate.score = distanceScore + goalScore + pathLengthScore;
                            } else {
                                candidate.score = -Infinity; // Unsafe direction
                            }
                        }
                        
                        // Find best avoidance direction
                        let bestAvoidance = null;
                        let bestScore = -Infinity;
                        for (const candidate of avoidanceCandidates) {
                            if (candidate.score > bestScore) {
                                bestScore = candidate.score;
                                bestAvoidance = candidate.angle;
                            }
                        }
                        
                        if (bestAvoidance !== null) {
                            avoidanceDirection = bestAvoidance;
                            
                            // Smooth deceleration when approaching obstacle
                            if (obsDist < OBSTACLE_DECELERATION_DISTANCE || isColliding) {
                                const targetSpeed = Math.max(MIN_APPROACH_VELOCITY, currentSpeed * DECELERATION_FACTOR * 0.9);
                                const decelSmoothing = 0.12; // Smooth deceleration
                                const speedRatio = targetSpeed / Math.max(currentSpeed, 0.01);
                                vx = vx * (1 - decelSmoothing) + vx * speedRatio * decelSmoothing;
                                vy = vy * (1 - decelSmoothing) + vy * speedRatio * decelSmoothing;
                            }
                            
                            // Smooth acceleration in the safe direction
                            const baseAcceleration = isColliding ? OBSTACLE_AVOIDANCE_STRENGTH * 2.5 : OBSTACLE_AVOIDANCE_STRENGTH * 1.8;
                            const accelerationStrength = baseAcceleration * (1 + (1 - obsDist / OBSTACLE_AVOIDANCE_DISTANCE) * 0.5); // Moderate strength when closer
                            const targetAvoidanceVx = Math.cos(avoidanceDirection) * accelerationStrength;
                            const targetAvoidanceVy = Math.sin(avoidanceDirection) * accelerationStrength;
                            
                            // Apply acceleration smoothly - much more gradual
                            const avoidanceSmoothing = isColliding ? 0.15 : 0.08; // Much smoother transitions
                            vx = vx * (1 - avoidanceSmoothing) + targetAvoidanceVx * avoidanceSmoothing;
                            vy = vy * (1 - avoidanceSmoothing) + targetAvoidanceVy * avoidanceSmoothing;
                            
                            // Smoothly ensure minimum velocity when colliding
                            if (isColliding && currentSpeed < BASE_VELOCITY * 0.3) {
                                const minSpeed = BASE_VELOCITY * 0.4;
                                const targetVx = (vx / Math.max(currentSpeed, 0.01)) * minSpeed;
                                const targetVy = (vy / Math.max(currentSpeed, 0.01)) * minSpeed;
                                const speedSmoothing = 0.1; // Smooth speed boost
                                vx = vx * (1 - speedSmoothing) + targetVx * speedSmoothing;
                                vy = vy * (1 - speedSmoothing) + targetVy * speedSmoothing;
                            }
                        } else if (isColliding) {
                            // No safe direction found but we're colliding - smoothly try opposite of current velocity
                            if (currentSpeed > 0.01) {
                                const oppositeAngle = Math.atan2(vy, vx) + Math.PI;
                                const escapeSpeed = BASE_VELOCITY * 0.8;
                                const targetVx = Math.cos(oppositeAngle) * escapeSpeed;
                                const targetVy = Math.sin(oppositeAngle) * escapeSpeed;
                                const escapeSmoothing = 0.2; // Smooth transition to escape direction
                                vx = vx * (1 - escapeSmoothing) + targetVx * escapeSmoothing;
                                vy = vy * (1 - escapeSmoothing) + targetVy * escapeSmoothing;
                            } else {
                                // No velocity - smoothly push directly away from obstacle
                                const pushX = obsDx / Math.max(obsDist, 0.1);
                                const pushY = obsDy / Math.max(obsDist, 0.1);
                                const targetVx = pushX * BASE_VELOCITY * 0.6;
                                const targetVy = pushY * BASE_VELOCITY * 0.6;
                                const pushSmoothing = 0.25; // Smooth push
                                vx = vx * (1 - pushSmoothing) + targetVx * pushSmoothing;
                                vy = vy * (1 - pushSmoothing) + targetVy * pushSmoothing;
                            }
                        }
                    } else {
                        // Not on collision course - might be recovering or far from obstacle
                        if (approachingObstacle || avoidingObstacle) {
                            // Re-accelerate after avoiding obstacle
                            const currentSpeed = Math.sqrt(vx * vx + vy * vy);
                            if (currentSpeed < baseVelocity) {
                                const acceleration = Math.min(ACCELERATION_FACTOR, baseVelocity / currentSpeed);
                                vx *= acceleration;
                                vy *= acceleration;
                            }
                        }
                        
                        if (obsDist > OBSTACLE_AVOIDANCE_DISTANCE * 0.9) {
                            approachingObstacle = false;
                            avoidingObstacle = false;
                            avoidanceDirection = null;
                        }
                    }
                } else {
                    // Not avoiding - reset flags
                    approachingObstacle = false;
                    avoidingObstacle = false;
                    avoidanceDirection = null;
                }
                
                // Calculate current speed and position change
                // Safety check: ensure vx and vy are valid numbers
                if (!isFinite(vx) || !isFinite(vy)) {
                    vx = 0;
                    vy = 0;
                }
                const currentSpeed = Math.sqrt(vx * vx + vy * vy);
                const positionChange = Math.sqrt((x - prevX) * (x - prevX) + (y - prevY) * (y - prevY));
                
                // Safety check: ensure calculated values are valid
                if (!isFinite(currentSpeed)) {
                    vx = BASE_VELOCITY * 0.5;
                    vy = BASE_VELOCITY * 0.5;
                }
                if (!isFinite(positionChange)) {
                    // Position change invalid, but not critical
                }
                
                const isMovingSlowly = currentSpeed < STUCK_VELOCITY_THRESHOLD;
                const isStuckInPlace = positionChange < STUCK_POSITION_THRESHOLD;
                
                // Detect if trying to move forward but blocked (stuck in front of obstacle)
                const goalDx = W - x;
                const goalDy = 0 - y;
                const goalAngle = Math.atan2(goalDy, goalDx);
                const currentAngle = Math.atan2(vy, vx);
                const angleToGoal = Math.abs(goalAngle - currentAngle);
                const isTryingToMoveForward = angleToGoal < Math.PI / 2 || angleToGoal > 3 * Math.PI / 2;
                
                // Handle collision physics - detect stuck state and force perpendicular escape
                if (isColliding && currentObstacle) {
                    const obsDx = x - currentObstacle.x;
                    const obsDy = y - currentObstacle.y;
                    const obsDist = Math.sqrt(obsDx*obsDx + obsDy*obsDy);
                    
                    // Safety check: ensure obsDist is valid
                    if (!isFinite(obsDist) || obsDist < 0.1) {
                        // Invalid distance - use default escape
                        vx = BASE_VELOCITY * 0.5;
                        vy = BASE_VELOCITY * 0.5;
                    } else {
                        // Calculate normal vector (away from obstacle center)
                        const normalX = obsDx / obsDist;
                        const normalY = obsDy / obsDist;
                    
                        // Check if velocity is parallel to obstacle surface (perpendicular to normal)
                        // This causes sliding along obstacle without escaping
                        const velocityDotNormal = vx * normalX + vy * normalY;
                        const velocityPerpX = vx - normalX * velocityDotNormal;
                        const velocityPerpY = vy - normalY * velocityDotNormal;
                        let perpSpeed = Math.sqrt(velocityPerpX*velocityPerpX + velocityPerpY*velocityPerpY);
                        let awaySpeed = Math.abs(velocityDotNormal);
                        
                        // Safety check: ensure calculated speeds are valid
                        if (!isFinite(perpSpeed)) {
                            perpSpeed = 0;
                        }
                        if (!isFinite(awaySpeed)) {
                            awaySpeed = 0;
                        }
                        
                        // Detect stuck: colliding AND (very slow OR not moving away OR sliding along surface)
                        const isTrulyStuck = isColliding && (
                            currentSpeed < BASE_VELOCITY * 0.15 || 
                            positionChange < 2 || 
                            (perpSpeed > awaySpeed * 2 && awaySpeed < BASE_VELOCITY * 0.1) // Sliding along surface
                        );
                        
                        if (isTrulyStuck) {
                            // Smooth perpendicular escape - push away from obstacle gradually
                            const escapeSpeed = BASE_VELOCITY * 1.0; // Moderate escape speed
                            
                            // Also try to find a better escape direction by testing nearby angles
                            let bestEscapeAngle = Math.atan2(normalY, normalX);
                            let bestEscapeDist = 0;
                            
                            // Test angles around the normal (perpendicular escape)
                            for (let angleOffset = -Math.PI/3; angleOffset <= Math.PI/3; angleOffset += Math.PI/12) {
                                const testAngle = Math.atan2(normalY, normalX) + angleOffset;
                                const testX = x + Math.cos(testAngle) * racerRadius * 5;
                                const testY = y + Math.sin(testAngle) * racerRadius * 5;
                                
                                const testSafe = !checkTCollision(testX, testY, currentObstacle.x, currentObstacle.y,
                                    currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                                
                                if (testSafe) {
                                    const testObsDx = testX - currentObstacle.x;
                                    const testObsDy = testY - currentObstacle.y;
                                    const testObsDist = Math.sqrt(testObsDx*testObsDx + testObsDy*testObsDy);
                                    
                                    if (testObsDist > bestEscapeDist) {
                                        bestEscapeDist = testObsDist;
                                        bestEscapeAngle = testAngle;
                                    }
                                }
                            }
                            
                            // Smoothly apply the best escape direction
                            const targetEscapeVx = Math.cos(bestEscapeAngle) * escapeSpeed;
                            const targetEscapeVy = Math.sin(bestEscapeAngle) * escapeSpeed;
                            const escapeSmoothing = 0.18; // Smooth escape transition
                            vx = vx * (1 - escapeSmoothing) + targetEscapeVx * escapeSmoothing;
                            vy = vy * (1 - escapeSmoothing) + targetEscapeVy * escapeSmoothing;
                            
                            // Safety check: ensure velocity is valid
                            if (!isFinite(vx) || !isFinite(vy)) {
                                const fallbackVx = normalX * escapeSpeed;
                                const fallbackVy = normalY * escapeSpeed;
                                vx = vx * 0.5 + fallbackVx * 0.5;
                                vy = vy * 0.5 + fallbackVy * 0.5;
                            }
                        } else if (currentSpeed < BASE_VELOCITY * 0.3) {
                            // Not fully stuck but slow - smoothly ensure we're moving away, not along
                            // Calculate target velocity away from obstacle
                            const targetAwaySpeed = Math.max(awaySpeed, BASE_VELOCITY * 0.5);
                            const targetVx = normalX * targetAwaySpeed;
                            const targetVy = normalY * targetAwaySpeed;
                            
                            // Smoothly blend with current velocity to maintain momentum
                            const blendSmoothing = 0.15; // Smooth blending
                            vx = vx * (1 - blendSmoothing) + targetVx * blendSmoothing;
                            vy = vy * (1 - blendSmoothing) + targetVy * blendSmoothing;
                            
                            // Safety check
                            if (!isFinite(vx) || !isFinite(vy)) {
                                vx = normalX * BASE_VELOCITY * 0.5;
                                vy = normalY * BASE_VELOCITY * 0.5;
                            }
                        } else {
                            // Has speed but might be sliding - smoothly ensure component away from obstacle
                            if (velocityDotNormal < BASE_VELOCITY * 0.2) {
                                // Not moving away enough - smoothly boost the away component
                                const boostStrength = BASE_VELOCITY * 0.3;
                                const boostSmoothing = 0.1; // Very smooth boost
                                vx = vx * (1 - boostSmoothing) + (vx + normalX * boostStrength) * boostSmoothing;
                                vy = vy * (1 - boostSmoothing) + (vy + normalY * boostStrength) * boostSmoothing;
                            }
                            
                            // Safety check
                            if (!isFinite(vx) || !isFinite(vy)) {
                                vx = normalX * BASE_VELOCITY * 0.5;
                                vy = normalY * BASE_VELOCITY * 0.5;
                            }
                        }
                    }
                    
                    // Reset stuck state - we're handling escape
                    stuck = false;
                    stuckFrames = 0;
                } else {
                    // Not colliding - clear stuck state
                    if (stuck) {
                        stuck = false;
                        stuckFrames = 0;
                        collisionDirection = null;
                        escapeBurstFrames = 0;
                    }
                    
                    // Normal movement (only when not stuck and not in escape burst)
                    // Only move if countdown is finished
                    if (escapeBurstFrames === 0 && countdown === null) {
                        vx += (targetVx - vx) * TARGET_SMOOTHING;
                        vy += (targetVy - vy) * TARGET_SMOOTHING;
                        
                        // Only apply goal pull when not stuck (goal pull prevents backward movement)
                        if (!stuck) {
                            const dx = W - x;
                            const dy = 0 - y;
                            const dist = Math.sqrt(dx*dx + dy*dy);
                            if (dist > 1) {
                                vx += (dx / dist) * GOAL_PULL_STRENGTH; 
                                vy += (dy / dist) * GOAL_PULL_STRENGTH;
                            }
                        }
                        
                        vx += (Math.random() - 0.5) * RANDOM_FACTOR;
                        vy += (Math.random() - 0.5) * RANDOM_FACTOR;
                        
                        vx *= DRAG; vy *= DRAG;
                    } else {
                        // During escape burst, maintain escape direction (ignore goal pull)
                        escapeBurstFrames--;
                        // Gradually reduce but maintain direction
                        const currentEscapeAngle = Math.atan2(vy, vx);
                        const burstStrength = ESCAPE_BURST_VELOCITY * (0.6 + (escapeBurstFrames / ESCAPE_BURST_DURATION) * 0.4);
                        vx = Math.cos(currentEscapeAngle) * burstStrength;
                        vy = Math.sin(currentEscapeAngle) * burstStrength;
                    }
                }

                // Calculate new position - safety check first
                if (!isFinite(vx)) vx = 0;
                if (!isFinite(vy)) vy = 0;
                if (!isFinite(x)) x = 0;
                if (!isFinite(y)) y = 0;
                
                const newX = x + vx;
                const newY = y + vy;
                
                // Check T-shaped collision before moving - prevent passing through
                if (currentObstacle) {
                    const willCollide = checkTCollision(newX, newY, currentObstacle.x, currentObstacle.y,
                        currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                    
                    if (willCollide) {
                        // Prevent passing through - don't move into obstacle
                        // Calculate normal from obstacle center to racer
                        const obsDx = x - currentObstacle.x;
                        const obsDy = y - currentObstacle.y;
                        const obsDist = Math.sqrt(obsDx*obsDx + obsDy*obsDy);
                        
                        // Initialize normal vectors with safe defaults
                        let normalX = 1;
                        let normalY = 0;
                        
                        if (obsDist > 0.1 && isFinite(obsDist)) {
                            normalX = obsDx / obsDist;
                            normalY = obsDy / obsDist;
                            
                            // Safety check: ensure normal is valid
                            if (!isFinite(normalX) || !isFinite(normalY)) {
                                normalX = 1;
                                normalY = 0;
                            }
                            
                            const dot = vx * normalX + vy * normalY;
                            
                            // Smoothly remove velocity component towards obstacle
                            if (dot > 0 && isFinite(dot)) {
                                const removalSmoothing = 0.3; // Smooth removal
                                const removedVx = normalX * dot;
                                const removedVy = normalY * dot;
                                vx = vx * (1 - removalSmoothing) + (vx - removedVx) * removalSmoothing;
                                vy = vy * (1 - removalSmoothing) + (vy - removedVy) * removalSmoothing;
                            }
                            
                            // If velocity is now too small or parallel to surface, smoothly add escape component
                            const currentSpeed = Math.sqrt(vx * vx + vy * vy);
                            if (isFinite(currentSpeed) && currentSpeed < BASE_VELOCITY * 0.2) {
                                // Smoothly add escape velocity perpendicular to obstacle
                                const escapeBoost = BASE_VELOCITY * 0.6;
                                const escapeSmoothing = 0.2; // Smooth escape boost
                                vx = vx * (1 - escapeSmoothing) + (vx + normalX * escapeBoost) * escapeSmoothing;
                                vy = vy * (1 - escapeSmoothing) + (vy + normalY * escapeBoost) * escapeSmoothing;
                            }
                        } else {
                            // Invalid distance - set default escape velocity
                            vx = BASE_VELOCITY * 0.5;
                            vy = BASE_VELOCITY * 0.5;
                        }
                        
                        // Final safety check on velocity
                        if (!isFinite(vx) || !isFinite(vy)) {
                            vx = BASE_VELOCITY * 0.5;
                            vy = BASE_VELOCITY * 0.5;
                        }
                        
                        // Try to find a safe position along the movement vector (small steps)
                        // But prioritize moving away from obstacle
                        const originalVx = newX - x;
                        const originalVy = newY - y;
                        let foundSafePosition = false;
                        let lastSafeX = x;
                        let lastSafeY = y;
                        
                        // First, try moving directly away from obstacle
                        if (obsDist > 0.1 && isFinite(obsDist) && isFinite(normalX) && isFinite(normalY)) {
                            const escapeX = x + normalX * racerRadius * 2;
                            const escapeY = y + normalY * racerRadius * 2;
                            
                            // Safety check on escape position
                            if (isFinite(escapeX) && isFinite(escapeY)) {
                                const escapeSafe = !checkTCollision(escapeX, escapeY, currentObstacle.x, currentObstacle.y,
                                    currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                                
                                if (escapeSafe) {
                                    x = escapeX;
                                    y = escapeY;
                                    foundSafePosition = true;
                                }
                            }
                        }
                        
                        // If direct escape didn't work, try along movement vector
                        if (!foundSafePosition && isFinite(originalVx) && isFinite(originalVy)) {
                            for (let step = 0.05; step <= 1.0; step += 0.05) {
                                const testX = x + originalVx * step;
                                const testY = y + originalVy * step;
                                
                                // Safety check on test position
                                if (!isFinite(testX) || !isFinite(testY)) {
                                    break;
                                }
                                
                                const testSafe = !checkTCollision(testX, testY, currentObstacle.x, currentObstacle.y,
                                    currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                                
                                if (testSafe) {
                                    lastSafeX = testX;
                                    lastSafeY = testY;
                                    foundSafePosition = true;
                                } else {
                                    // Hit obstacle - use last safe position
                                    break;
                                }
                            }
                            
                            if (foundSafePosition && isFinite(lastSafeX) && isFinite(lastSafeY)) {
                                // Move to the furthest safe position
                                x = lastSafeX;
                                y = lastSafeY;
                            } else {
                                // Can't move forward at all - stay at current position
                                // Escape velocity has been set above
                            }
                        }
                    } else {
                        // Safe to move to new position
                        x = newX;
                        y = newY;
                    }
                } else {
                    // No obstacle, safe to move
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

                return { ...r, x, y, vx, vy, targetVx, targetVy, angle, tail, stuck, stuckFrames, collisionDirection, escapeBurstFrames, prevX: x, prevY: y, approachingObstacle, avoidingObstacle, avoidanceDirection, baseVelocity };
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
                vx: r.targetVx || ((Math.random() * BASE_VELOCITY) + BASE_VELOCITY/2),
                vy: r.targetVy || (-(Math.random() * BASE_VELOCITY) - BASE_VELOCITY/2)
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
                    const delay = Math.random() * 1300 + 200; 
                    
                    const timer = setTimeout(() => {
                        setRacers(prevRacers => {
                            const currentRacer = prevRacers.find(r => r.id === racerId);
                            if (!currentRacer || currentRacer.finished) {
                                return prevRacers;
                            }
                            
                            const topRightBias = 0.3;
                            const randomVx = (Math.random() * BASE_VELOCITY * 2) - BASE_VELOCITY;
                            const randomVy = (Math.random() * BASE_VELOCITY * 2) - BASE_VELOCITY;
                            
                            const newTargetVx = randomVx + (BASE_VELOCITY * topRightBias);
                            const newTargetVy = randomVy - (BASE_VELOCITY * topRightBias);
                            
                            return prevRacers.map(r => 
                                r.id === racerId ? { ...r, targetVx: newTargetVx, targetVy: newTargetVy } : r
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
                            <h1 style={{ fontSize: '42px', fontWeight: '900', background: 'linear-gradient(135deg, #ffffff 0%, #a0aec0 100%)', WebkitBackgroundClip: 'text', WebkitTextFillColor: 'transparent', backgroundClip: 'text', marginBottom: '8px', letterSpacing: '-0.03em' }}>MICRO DERBY</h1>
                            <p style={{ fontSize: '13px', color: '#9ca3af', letterSpacing: '0.05em', textTransform: 'uppercase' }}>Experimental Racing Simulation</p>
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
                                {statusText.includes('DEFEAT') ? 'EXPERIMENT FAILED' : 'EXPERIMENT SUCCESS'}
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