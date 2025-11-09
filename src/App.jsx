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
const OBSTACLE_AVOIDANCE_DISTANCE = 150; // Distance to start avoiding obstacle
const OBSTACLE_AVOIDANCE_STRENGTH = 0.5; // Strength of avoidance force
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
    const velocityChangeTimersRef = useRef([]);

    const initializeRace = useCallback((width, height) => {
        setDust(generateDust(width, height));
        setFinishedRankings([]);
        setStatusText("");

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

        return HORSES_CONFIG.map((config) => {
            const startX = Math.random() * (width * 0.10);
            const startY = (height * 0.90) + (Math.random() * (height * 0.10));
            const vx = (Math.random() * BASE_VELOCITY) + BASE_VELOCITY/2;
            const vy = -(Math.random() * BASE_VELOCITY) - BASE_VELOCITY/2;

            return {
                ...config,
                x: startX, y: startY, 
                vx, vy,
                targetVx: vx, targetVy: vy,
                angle: Math.atan2(vy, vx),
                tail: [],
                finished: false,
                finalX: 0, finalY: 0,
                stuck: false, // Whether racer is stuck in obstacle
                stuckFrames: 0, // Frames spent stuck
                collisionDirection: null, // Direction of collision (angle in radians)
                escapeBurstFrames: 0, // Frames remaining in escape burst
                prevX: startX, // Previous X position for stuck detection
                prevY: startY // Previous Y position for stuck detection
            };
        });
    }, []);

    const checkResultAndSignal = useCallback(async (rankings) => {
        const madeTheCut = rankings.includes(bet);
        
        if (madeTheCut) {
            const rank = rankings.indexOf(bet) + 1;
            setStatusText(`SAFE: Finished #${rank}. No signal sent.`);
        } else {
            setStatusText('DEFEAT: Did not make the cut. Transmitting penalty... ⚡');
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
        const canvas = canvasRef.current;
        if (!canvas || gameState !== 'RACING') return;

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

                let { x, y, vx, vy, targetVx, targetVy, angle, tail, stuck, stuckFrames, collisionDirection, escapeBurstFrames } = r;
                
                // Track previous position to detect if stuck in place
                const prevX = r.prevX !== undefined ? r.prevX : x;
                const prevY = r.prevY !== undefined ? r.prevY : y;

                // T-shaped collision detection with physical collision
                const racerRadius = Math.max(RACER_WIDTH, RACER_HEIGHT);
                const obstacleSize = currentObstacle ? currentObstacle.size : OBSTACLE_SIZE_MIN;
                const verticalLength = obstacleSize * 0.6;
                const horizontalLength = obstacleSize * 0.4;
                const barThickness = obstacleSize * 0.08;
                
                // Obstacle avoidance - detect obstacle before collision
                if (currentObstacle && !stuck && escapeBurstFrames === 0) {
                    const obsDx = x - currentObstacle.x;
                    const obsDy = y - currentObstacle.y;
                    const obsDist = Math.sqrt(obsDx*obsDx + obsDy*obsDy);
                    
                    // Check if approaching obstacle
                    if (obsDist < OBSTACLE_AVOIDANCE_DISTANCE && obsDist > racerRadius * 2) {
                        // Check if current trajectory would hit obstacle
                        const futureX = x + vx * 5;
                        const futureY = y + vy * 5;
                        const wouldHit = checkTCollision(futureX, futureY, currentObstacle.x, currentObstacle.y,
                            currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                        
                        if (wouldHit) {
                            // Apply avoidance force perpendicular to obstacle direction
                            const avoidAngle = Math.atan2(obsDy, obsDx) + Math.PI / 2;
                            // Choose direction that's more towards goal
                            const goalDx = W - x;
                            const goalDy = 0 - y;
                            const goalAngle = Math.atan2(goalDy, goalDx);
                            const angleDiff1 = Math.abs(avoidAngle - goalAngle);
                            const angleDiff2 = Math.abs((avoidAngle + Math.PI) - goalAngle);
                            const finalAvoidAngle = angleDiff1 < angleDiff2 ? avoidAngle : avoidAngle + Math.PI;
                            
                            vx += Math.cos(finalAvoidAngle) * OBSTACLE_AVOIDANCE_STRENGTH;
                            vy += Math.sin(finalAvoidAngle) * OBSTACLE_AVOIDANCE_STRENGTH;
                        }
                    }
                }
                
                // Check if currently colliding
                let isColliding = false;
                if (currentObstacle) {
                    isColliding = checkTCollision(x, y, currentObstacle.x, currentObstacle.y, 
                        currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                }
                
                // Calculate current speed and position change
                const currentSpeed = Math.sqrt(vx * vx + vy * vy);
                const positionChange = Math.sqrt((x - prevX) * (x - prevX) + (y - prevY) * (y - prevY));
                const isMovingSlowly = currentSpeed < STUCK_VELOCITY_THRESHOLD;
                const isStuckInPlace = positionChange < STUCK_POSITION_THRESHOLD;
                
                // Detect if trying to move forward but blocked (stuck in front of obstacle)
                const goalDx = W - x;
                const goalDy = 0 - y;
                const goalAngle = Math.atan2(goalDy, goalDx);
                const currentAngle = Math.atan2(vy, vx);
                const angleToGoal = Math.abs(goalAngle - currentAngle);
                const isTryingToMoveForward = angleToGoal < Math.PI / 2 || angleToGoal > 3 * Math.PI / 2;
                
                // Handle collision physics
                if (isColliding) {
                    if (!stuck) {
                        // Just collided - get stuck and record collision direction
                        stuck = true;
                        stuckFrames = 0;
                        
                        // Calculate collision direction (direction racer was moving when it hit)
                        if (currentSpeed > 0.01) {
                            collisionDirection = Math.atan2(vy, vx);
                        } else {
                            // If no velocity, use direction from obstacle center
                            const obsDx = x - currentObstacle.x;
                            const obsDy = y - currentObstacle.y;
                            collisionDirection = Math.atan2(obsDy, obsDx);
                        }
                        vx = 0;
                        vy = 0;
                    } else {
                        // Already stuck - increment stuck frames
                        stuckFrames++;
                        
                        // Check if stuck in front of obstacle (low velocity + stuck in place + trying to move forward)
                        const isStuckInFront = isMovingSlowly && isStuckInPlace && isTryingToMoveForward;
                        
                        // Trigger escape if: stuck for duration AND (very slow OR stuck in front)
                        const shouldEscape = stuckFrames >= STUCK_DURATION && (isMovingSlowly || isStuckInFront);
                        
                        if (shouldEscape && escapeBurstFrames === 0 && collisionDirection !== null) {
                            // Intelligent escape: calculate best escape direction (ALLOW BACKWARD MOVEMENT)
                            const obsDx = x - currentObstacle.x;
                            const obsDy = y - currentObstacle.y;
                            const obsDist = Math.sqrt(obsDx*obsDx + obsDy*obsDy);
                            
                            // Try multiple escape strategies (including backward movement)
                            let escapeCandidates = [];
                            
                            // Strategy 1: Perpendicular to collision (left/right) - allows sideways escape
                            const escapeAngle1 = collisionDirection + Math.PI / 2;
                            const escapeAngle2 = collisionDirection - Math.PI / 2;
                            escapeCandidates.push({ angle: escapeAngle1, priority: 1 });
                            escapeCandidates.push({ angle: escapeAngle2, priority: 1 });
                            
                            // Strategy 2: Directly away from obstacle (BACKWARD from goal, but necessary to escape)
                            const awayAngle = Math.atan2(obsDy, obsDx);
                            escapeCandidates.push({ angle: awayAngle, priority: 2 });
                            
                            // Strategy 3: Opposite to collision direction (BACKWARD - what they came from)
                            const oppositeAngle = collisionDirection + Math.PI;
                            escapeCandidates.push({ angle: oppositeAngle, priority: 3 });
                            
                            // Strategy 4: Perpendicular away from goal (if goal pull is blocking)
                            const perpendicularToGoal1 = goalAngle + Math.PI / 2;
                            const perpendicularToGoal2 = goalAngle - Math.PI / 2;
                            escapeCandidates.push({ angle: perpendicularToGoal1, priority: 2 });
                            escapeCandidates.push({ angle: perpendicularToGoal2, priority: 2 });
                            
                            // Test each candidate and find the best one
                            let bestEscape = null;
                            let bestScore = -Infinity;
                            
                            for (const candidate of escapeCandidates) {
                                const testX = x + Math.cos(candidate.angle) * racerRadius * 3;
                                const testY = y + Math.sin(candidate.angle) * racerRadius * 3;
                                
                                // Check if this direction is safe (no collision)
                                const isSafe = !checkTCollision(testX, testY, currentObstacle.x, currentObstacle.y,
                                    currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                                
                                if (isSafe) {
                                    // Score based on: safety, distance from obstacle, and priority
                                    const testObsDx = testX - currentObstacle.x;
                                    const testObsDy = testY - currentObstacle.y;
                                    const testObsDist = Math.sqrt(testObsDx*testObsDx + testObsDy*testObsDy);
                                    
                                    // Prefer directions that move away from obstacle (most important)
                                    const distanceScore = testObsDist * 2;
                                    // Lower priority number = better (but distance is more important)
                                    const priorityScore = (4 - candidate.priority) * 5;
                                    // Small preference for directions that eventually help reach goal (but not required)
                                    const goalAlignment = Math.cos(candidate.angle - goalAngle);
                                    const goalScore = goalAlignment * 0.5; // Small weight
                                    
                                    const totalScore = distanceScore + priorityScore + goalScore;
                                    
                                    if (totalScore > bestScore) {
                                        bestScore = totalScore;
                                        bestEscape = candidate.angle;
                                    }
                                }
                            }
                            
                            // Use best escape, or fallback to directly away from obstacle (backward if needed)
                            const finalEscapeAngle = bestEscape !== null ? bestEscape : awayAngle;
                            
                            vx = Math.cos(finalEscapeAngle) * ESCAPE_BURST_VELOCITY;
                            vy = Math.sin(finalEscapeAngle) * ESCAPE_BURST_VELOCITY;
                            escapeBurstFrames = ESCAPE_BURST_DURATION;
                        } else if (escapeBurstFrames > 0) {
                            // Maintain escape burst - continue in current escape direction
                            escapeBurstFrames--;
                            // Gradually reduce burst strength but maintain direction
                            const currentEscapeAngle = Math.atan2(vy, vx);
                            const burstStrength = ESCAPE_BURST_VELOCITY * (0.6 + (escapeBurstFrames / ESCAPE_BURST_DURATION) * 0.4);
                            vx = Math.cos(currentEscapeAngle) * burstStrength;
                            vy = Math.sin(currentEscapeAngle) * burstStrength;
                        } else if (stuckFrames < STUCK_DURATION) {
                            // Still in initial stuck period - no movement
                            vx = 0;
                            vy = 0;
                        } else {
                            // Fallback: try to move away from obstacle
                            const obsDx = x - currentObstacle.x;
                            const obsDy = y - currentObstacle.y;
                            const obsDist = Math.sqrt(obsDx*obsDx + obsDy*obsDy);
                            
                            if (obsDist > 0.1) {
                                const escapeX = (obsDx / obsDist) * WIGGLE_STRENGTH * 2;
                                const escapeY = (obsDy / obsDist) * WIGGLE_STRENGTH * 2;
                                vx = escapeX;
                                vy = escapeY;
                            } else {
                                // Random wiggle if at center
                                const wiggleAngle = Math.random() * Math.PI * 2;
                                vx = Math.cos(wiggleAngle) * WIGGLE_STRENGTH;
                                vy = Math.sin(wiggleAngle) * WIGGLE_STRENGTH;
                            }
                        }
                    }
                } else {
                    // Not colliding - if was stuck, clear stuck state
                    if (stuck) {
                        stuck = false;
                        stuckFrames = 0;
                        collisionDirection = null;
                        escapeBurstFrames = 0;
                    }
                    
                    // Normal movement (only when not stuck and not in escape burst)
                    if (escapeBurstFrames === 0) {
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

                // Calculate new position
                const newX = x + vx;
                const newY = y + vy;
                
                // Check T-shaped collision before moving - ALWAYS check, even when stuck
                let canMove = true;
                if (currentObstacle) {
                    const willCollide = checkTCollision(newX, newY, currentObstacle.x, currentObstacle.y,
                        currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                    
                    if (willCollide) {
                        canMove = false;
                        // Prevent any movement into the obstacle
                        vx = 0;
                        vy = 0;
                        
                        // If stuck and wiggling, try to find escape path
                        if (stuck && stuckFrames >= STUCK_DURATION) {
                            // Try multiple escape directions
                            let foundEscape = false;
                            for (let i = 0; i < 8; i++) {
                                const escapeAngle = (Math.PI * 2 * i) / 8;
                                const safeDistance = racerRadius * 3;
                                const testX = x + Math.cos(escapeAngle) * safeDistance;
                                const testY = y + Math.sin(escapeAngle) * safeDistance;
                                
                                // Check if escape position is safe
                                const escapeSafe = !checkTCollision(testX, testY, currentObstacle.x, currentObstacle.y,
                                    currentObstacle.rotation, verticalLength, horizontalLength, barThickness, racerRadius);
                                
                                if (escapeSafe) {
                                    // Move towards escape position (small step)
                                    const stepSize = racerRadius * 0.5;
                                    x = x + Math.cos(escapeAngle) * stepSize;
                                    y = y + Math.sin(escapeAngle) * stepSize;
                                    vx = Math.cos(escapeAngle) * WIGGLE_STRENGTH;
                                    vy = Math.sin(escapeAngle) * WIGGLE_STRENGTH;
                                    foundEscape = true;
                                    break;
                                }
                            }
                            
                            if (!foundEscape) {
                                // Can't escape yet, stay in place
                                vx = 0;
                                vy = 0;
                            }
                        }
                    }
                }
                
                // Only update position if no collision detected and we're not handling escape
                if (canMove) {
                    x = newX;
                    y = newY;
                }

                if (x < 0) { x = 0; vx *= -WALL_BOUNCE_DAMPING; }
                if (x > W) { x = W; vx *= -WALL_BOUNCE_DAMPING; }
                if (y < 0) { y = 0; vy *= -WALL_BOUNCE_DAMPING; }
                if (y > H) { y = H; vy *= -WALL_BOUNCE_DAMPING; }
                
                angle = Math.atan2(vy, vx);

                tail = [...tail, { x, y }];
                if (tail.length > TAIL_LENGTH) tail.shift();

                const distToCorner = Math.sqrt(Math.pow(W - x, 2) + Math.pow(0 - y, 2));
                if (distToCorner < GOAL_RADIUS) {
                    return { ...r, x, y, vx: 0, vy: 0, finished: true, finalX: x, finalY: y };
                }

                return { ...r, x, y, vx, vy, targetVx, targetVy, angle, tail, stuck, stuckFrames, collisionDirection, escapeBurstFrames, prevX: x, prevY: y };
            });

            newRacers.forEach(r => {
                if (r.finished && !currentFinishers.includes(r.id)) {
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
    }, [gameState, finishedRankings, difficulty, obstacle]);

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
        ctx.fillStyle = 'rgba(255, 255, 255, 0.05)'; ctx.fill();
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)'; ctx.lineWidth = 2; ctx.stroke();
        
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
    }, [racers, dust, obstacle, bet]);

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
                <div style={{ position: 'fixed', top: 0, left: 0, right: 0, bottom: 0, zIndex: 5000, pointerEvents: 'none' }}>
                    {racers.map(r => <RacerOutline key={r.id} racer={r} />)}
                </div>
            )}

            <div style={{ position: 'fixed', top: 0, left: 0, right: 0, bottom: 0, zIndex: 10000, pointerEvents: 'none', display: 'flex', alignItems: 'center', justifyContent: 'center', padding: '20px' }}>
                {gameState === 'SETUP' && (
                    <div style={{ pointerEvents: 'auto', backgroundColor: 'rgba(17, 24, 39, 0.95)', backdropFilter: 'blur(12px)', borderRadius: '24px', border: '1px solid rgba(255, 255, 255, 0.1)', boxShadow: '0 25px 50px -12px rgba(0, 0, 0, 0.5)', padding: '32px', maxWidth: '600px', width: '100%' }}>
                        <h1 style={{ fontSize: '36px', fontWeight: '900', color: '#ffffff', textAlign: 'center', marginBottom: '32px', letterSpacing: '-0.02em' }}>MICRO DERBY</h1>
                        
                        <div style={{ backgroundColor: 'rgba(0, 0, 0, 0.4)', padding: '24px', borderRadius: '16px', marginBottom: '32px', border: '1px solid rgba(255, 255, 255, 0.05)' }}>
                            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '16px', color: '#d1d5db', fontSize: '12px', fontFamily: 'monospace', textTransform: 'uppercase', letterSpacing: '0.1em' }}>
                                <span>Risk Level</span>
                                <span style={{ color: '#818cf8', fontWeight: 'bold' }}>TOP {difficulty} Finishers</span>
                            </div>
                            <input type="range" min="1" max="8" step="1" value={difficulty} onChange={(e) => setDifficulty(Number(e.target.value))} style={{ width: '100%', height: '8px', borderRadius: '4px', backgroundColor: '#374151', outline: 'none', cursor: 'pointer', WebkitAppearance: 'none', appearance: 'none' }} />
                            <p style={{ textAlign: 'center', color: '#6b7280', fontSize: '12px', marginTop: '12px', marginBottom: 0 }}>
                                Race ends when {difficulty} racers reach the goal. You must be one of them.
                            </p>
                        </div>

                        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '16px' }}>
                            {HORSES_CONFIG.map(h => (
                                <button key={h.id} onClick={() => { setBet(h.id); setRacers(initializeRace(window.innerWidth, window.innerHeight)); gameStateRef.current = 'RACING'; setGameState('RACING'); }} onMouseEnter={(e) => { e.currentTarget.style.transform = 'scale(1.05)'; e.currentTarget.style.backgroundColor = 'rgba(55, 65, 81, 0.7)'; }} onMouseLeave={(e) => { e.currentTarget.style.transform = 'scale(1)'; e.currentTarget.style.backgroundColor = 'rgba(31, 41, 55, 0.5)'; }} style={{ padding: '16px', borderRadius: '16px', border: `2px solid ${bet === h.id ? h.color : 'transparent'}`, backgroundColor: 'rgba(31, 41, 55, 0.5)', cursor: 'pointer', transition: 'all 0.2s', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', position: 'relative', overflow: 'hidden' }}>
                                    <span style={{ fontSize: '36px', marginBottom: '8px', filter: bet === h.id ? 'grayscale(0)' : 'grayscale(1)' }}>{h.emoji}</span>
                                    <div style={{ position: 'absolute', bottom: 0, left: 0, right: 0, height: '4px', backgroundColor: h.color, opacity: 0.5 }} />
                                </button>
                            ))}
                        </div>
                    </div>
                )}

                {gameState === 'FINISHED' && (
                    <div style={{ pointerEvents: 'auto', backgroundColor: 'rgba(17, 24, 39, 0.95)', backdropFilter: 'blur(12px)', borderRadius: '24px', border: '1px solid rgba(255, 255, 255, 0.1)', boxShadow: '0 25px 50px -12px rgba(0, 0, 0, 0.5)', padding: '32px', maxWidth: '500px', width: '100%', textAlign: 'center' }}>
                        <h2 style={{ fontSize: '28px', fontWeight: '900', color: '#ffffff', marginBottom: '24px' }}>EXPERIMENT COMPLETE</h2>
                        
                        <div style={{ marginBottom: '32px', textAlign: 'left', backgroundColor: 'rgba(0, 0, 0, 0.3)', padding: '16px', borderRadius: '12px', maxHeight: '240px', overflowY: 'auto' }}>
                            <h3 style={{ fontSize: '10px', fontWeight: 'bold', color: '#6b7280', marginBottom: '12px', textTransform: 'uppercase', letterSpacing: '0.1em' }}>Qualified Specimens (Top {difficulty})</h3>
                            <ol style={{ listStyle: 'none', padding: 0, margin: 0 }}>
                                {finishedRankings.map((rId, idx) => {
                                    const racer = HORSES_CONFIG.find(h => h.id === rId);
                                    const isMyBet = rId === bet;
                                    return (
                                        <li key={rId} style={{ display: 'flex', alignItems: 'center', padding: '8px', borderRadius: '8px', marginBottom: '8px', backgroundColor: isMyBet ? 'rgba(99, 102, 241, 0.3)' : 'transparent', border: isMyBet ? '1px solid rgba(99, 102, 241, 0.5)' : 'none' }}>
                                            <span style={{ color: '#6b7280', fontFamily: 'monospace', width: '24px' }}>{idx + 1}.</span>
                                            <span style={{ marginRight: '8px', fontSize: '20px' }}>{racer.emoji}</span>
                                            <span style={{ color: isMyBet ? '#ffffff' : '#d1d5db', fontWeight: isMyBet ? 'bold' : 'normal', flex: 1 }}>{racer.name}</span>
                                            {isMyBet && <span style={{ fontSize: '10px', backgroundColor: '#6366f1', color: '#ffffff', padding: '4px 8px', borderRadius: '12px', marginLeft: 'auto' }}>YOU</span>}
                                        </li>
                                    );
                                })}
                            </ol>
                        </div>

                        <div style={{ fontSize: '18px', fontWeight: 'bold', marginBottom: '24px', color: statusText.includes('DEFEAT') ? '#f87171' : '#4ade80' }}>{statusText}</div>

                        <button onClick={() => { gameStateRef.current = 'SETUP'; setGameState('SETUP'); }} onMouseEnter={(e) => e.currentTarget.style.backgroundColor = '#e5e7eb'} onMouseLeave={(e) => e.currentTarget.style.backgroundColor = '#ffffff'} style={{ width: '100%', backgroundColor: '#ffffff', color: '#000000', padding: '16px 32px', borderRadius: '12px', fontWeight: 'bold', fontSize: '14px', letterSpacing: '0.05em', border: 'none', cursor: 'pointer', transition: 'background-color 0.2s' }}>RUN NEW TEST</button>
                    </div>
                )}
            </div>
        </div>
    );
};

export default App;