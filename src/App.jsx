import React, { useState, useEffect, useRef, useCallback, useMemo } from 'react';

// --- CONFIGURATION CONSTANTS ---
// IMPORTANT: Replace with the actual IP address of your ESP32 server
const ESP32_IP_ADDRESS = '192.168.4.1'; 
const API_LOSE_ENDPOINT = `http://${ESP32_IP_ADDRESS}/lose`;

const RACER_SIZE = 3;
const TAIL_LENGTH = 20; // Shorter tail
const TRACK_COLOR = '#1a1a1a'; // Dark greyish with dirty black
const RACER_WIDTH = 5; // Width for oval shape
const RACER_HEIGHT = 3; // Height for oval shape

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

// --- HELPER FUNCTIONS ---
const generateDust = (width, height, count = 80) => {
    return Array.from({ length: count }, () => ({
        x: Math.random() * width,
        y: Math.random() * height,
        size: Math.random() * 2,
        opacity: Math.random() * 0.2
    }));
};

// --- CORE APP COMPONENT ---
const App = () => {
    const canvasRef = useRef(null);
    const animationFrameRef = useRef();
    const gameStateRef = useRef('SETUP');
    const [gameState, setGameState] = useState('SETUP'); // SETUP, RACING, FINISHED
    const [dust, setDust] = useState([]);

    // Game state data
    const [racers, setRacers] = useState([]);
    const [finishedRankings, setFinishedRankings] = useState([]); // Stores IDs in order of finish
    const [bet, setBet] = useState(null);
    const [difficulty, setDifficulty] = useState(4); // How many need to finish before race ends
    const [statusText, setStatusText] = useState("");
    const velocityChangeTimersRef = useRef([]); // Store timers for random velocity changes

    // --- LOGIC: INITIALIZATION ---
    const initializeRace = useCallback((width, height) => {
        setDust(generateDust(width, height));
        setFinishedRankings([]);
        setStatusText("");

        // Clear any existing timers
        velocityChangeTimersRef.current.forEach(timer => clearTimeout(timer));
        velocityChangeTimersRef.current = [];

        return HORSES_CONFIG.map((config) => {
            // Spawn Zone: Smaller bottom-left area (0-10% width, 90-100% height)
            const startX = Math.random() * (width * 0.10);
            const startY = (height * 0.90) + (Math.random() * (height * 0.10));
            
            // Slower initial velocity for ~5 second races
            const vx = (Math.random() * 0.5) + 0.3;
            const vy = -(Math.random() * 0.5) - 0.3;

            return {
                ...config,
                x: startX, y: startY, vx, vy,
                tail: [],
                finished: false,
                finalX: 0, finalY: 0 // To freeze them at finish line
            };
        });
    }, []);

    // --- LOGIC: PENALTY TRIGGER ---
    const checkResultAndSignal = useCallback(async (rankings) => {
        // Did the user's bet make it into the finishedRankings?
        const madeTheCut = rankings.includes(bet);
        
        if (madeTheCut) {
            const rank = rankings.indexOf(bet) + 1;
            setStatusText(`SAFE: Finished #${rank}. No signal sent.`);
        } else {
            setStatusText('DEFEAT: Did not make the cut. Transmitting penalty... \u26A1');
            try {
                fetch(API_LOSE_ENDPOINT, { method: 'POST' }).catch(e => console.warn("Signal dispatched quietly"));
            } catch (error) {
                setStatusText('CONNECTION FAILED to ESP32');
            }
        }
    }, [bet]);

    // --- LOGIC: MAIN GAME LOOP ---
    const gameLoop = useCallback(() => {
        const canvas = canvasRef.current;
        if (!canvas || gameState !== 'RACING') return;

        const W = canvas.width;
        const H = canvas.height;
        const GOAL_RADIUS = Math.max(W, H) * 0.15; // Smaller goal area - 15% of largest screen dimension

        setRacers(prevRacers => {
            let currentFinishers = [...finishedRankings];

            const newRacers = prevRacers.map(r => {
                if (r.finished) return r; // Frozen if finished

                let { x, y, vx, vy, tail } = r;

                // Movement physics (Drift to top-right + Brownian motion) - Slower
                const dx = W - x;
                const dy = 0 - y;
                const dist = Math.sqrt(dx*dx + dy*dy);
                vx += (dx / dist) * 0.02; // Reduced pull strength
                vy += (dy / dist) * 0.02;
                vx += (Math.random() - 0.5) * 0.3; // Reduced random motion
                vy += (Math.random() - 0.5) * 0.3;
                vx *= 0.998; vy *= 0.998; // Slight drag

                x += vx; y += vy;

                // Wall Bounce
                if (x < 0) { x = 0; vx *= -0.8; }
                if (x > W) { x = W; vx *= -0.8; }
                if (y < 0) { y = 0; vy *= -0.8; }
                if (y > H) { y = H; vy *= -0.8; }

                tail = [...tail, { x, y }];
                if (tail.length > TAIL_LENGTH) tail.shift();

                // Goal Check
                const distToCorner = Math.sqrt(Math.pow(W - x, 2) + Math.pow(0 - y, 2));
                if (distToCorner < GOAL_RADIUS) {
                    return { ...r, x, y, vx, vy, tail, finished: true, finalX: x, finalY: y };
                }

                return { ...r, x, y, vx, vy, tail };
            });

            // Identify new finishers this frame
            newRacers.forEach(r => {
                if (r.finished && !currentFinishers.includes(r.id)) {
                    currentFinishers.push(r.id);
                }
            });

            // Update rankings state if it changed
            if (currentFinishers.length !== finishedRankings.length) {
                setFinishedRankings(currentFinishers);
            }

            // Race End Condition matched?
            if (currentFinishers.length >= difficulty) {
                gameStateRef.current = 'FINISHED';
                setGameState('FINISHED');
            }

            return newRacers;
        });

        if (gameState === 'RACING') {
            animationFrameRef.current = requestAnimationFrame(gameLoop);
        }
    }, [gameState, finishedRankings, difficulty]);

    // --- RENDERING ---
    const draw = useCallback(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const W = canvas.width; const H = canvas.height;

        // Background
        ctx.fillStyle = TRACK_COLOR;
        ctx.fillRect(0, 0, W, H);
        dust.forEach(d => {
            ctx.fillStyle = `rgba(255,255,255,${d.opacity})`;
            ctx.beginPath(); ctx.arc(d.x, d.y, d.size, 0, Math.PI * 2); ctx.fill();
        });

        // Goal Zone - Smaller
        const GOAL_RADIUS = Math.max(W, H) * 0.15;
        ctx.beginPath(); ctx.moveTo(W, 0);
        ctx.arc(W, 0, GOAL_RADIUS, Math.PI / 2, Math.PI, false);
        ctx.closePath();
        ctx.fillStyle = 'rgba(255, 255, 255, 0.05)'; ctx.fill();
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)'; ctx.lineWidth = 2; ctx.stroke();
        
        // Racers
        racers.forEach(r => {
            // Draw tail
            ctx.beginPath();
            for (let i = 0; i < r.tail.length; i++) {
                // If finished, tail fades out faster
                const opacity = r.finished ? 0.1 : (i / TAIL_LENGTH) * 0.4;
                ctx.strokeStyle = `rgba(255, 255, 255, ${opacity})`;
                i === 0 ? ctx.moveTo(r.tail[i].x, r.tail[i].y) : ctx.lineTo(r.tail[i].x, r.tail[i].y);
            }
            ctx.stroke();

            // Draw oval racer body
            ctx.fillStyle = r.finished ? r.color : '#ffffff';
            ctx.save();
            ctx.translate(r.x, r.y);
            // Calculate angle from velocity for orientation
            const angle = Math.atan2(r.vy, r.vx);
            ctx.rotate(angle);
            ctx.beginPath();
            ctx.ellipse(0, 0, r.finished ? RACER_WIDTH : RACER_WIDTH/1.5, r.finished ? RACER_HEIGHT : RACER_HEIGHT/1.5, 0, 0, Math.PI * 2);
            ctx.fill();
            ctx.restore();
        });
    }, [racers, dust]);

    // --- EFFECTS ---
    useEffect(() => {
        const resizeCanvas = () => {
            if (canvasRef.current) {
                canvasRef.current.width = window.innerWidth;
                canvasRef.current.height = window.innerHeight;
                if (gameState === 'SETUP') setRacers(initializeRace(window.innerWidth, window.innerHeight));
            }
        };
        resizeCanvas();
        window.addEventListener('resize', resizeCanvas);
        return () => window.removeEventListener('resize', resizeCanvas);
    }, [initializeRace, gameState]);

    // --- EFFECT: Random velocity changes at random intervals ---
    useEffect(() => {
        if (gameState !== 'RACING' || racers.length === 0) {
            // Clear all timers when not racing or no racers
            velocityChangeTimersRef.current.forEach(timer => clearTimeout(timer));
            velocityChangeTimersRef.current = [];
            return;
        }

        // Set up random velocity changes for each racer - Individual timers per racer
        racers.forEach((racer) => {
            const scheduleNextChange = (racerId) => {
                // Random interval between 0.2 and 0.6 seconds (200-600ms)
                const delay = Math.random() * 400 + 200;
                
                const timer = setTimeout(() => {
                    // Check game state and racer status using functional update
                    setRacers(prevRacers => {
                        const currentRacer = prevRacers.find(r => r.id === racerId);
                        if (!currentRacer || currentRacer.finished) {
                            return prevRacers; // Don't change if finished or not found
                        }
                        
                        // Give new random velocity - Slower for ~5 second races
                        const newVx = (Math.random() * 0.5) + 0.3;
                        const newVy = -(Math.random() * 0.5) - 0.3;
                        
                        return prevRacers.map(r => 
                            r.id === racerId ? { ...r, vx: newVx, vy: newVy } : r
                        );
                    });
                    
                    // Schedule next change only if still racing
                    if (gameStateRef.current === 'RACING') {
                        scheduleNextChange(racerId);
                    }
                }, delay);
                
                velocityChangeTimersRef.current.push(timer);
            };
            
            scheduleNextChange(racer.id);
        });

        return () => {
            velocityChangeTimersRef.current.forEach(timer => clearTimeout(timer));
            velocityChangeTimersRef.current = [];
        };
    }, [gameState, racers.length]); // Re-run when racers are initialized or game state changes

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
        draw();
        if (gameState === 'RACING') animationFrameRef.current = requestAnimationFrame(draw);
    }, [draw, gameState]);

    // --- UI COMPONENTS ---
    const RacerOutline = ({ racer }) => {
        if (!canvasRef.current) return null;
        const canvas = canvasRef.current;
        
        // Canvas coordinates match screen coordinates since canvas.width = window.innerWidth
        // and canvas.height = window.innerHeight
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

    // UI Styles - Using inline styles to ensure visibility
    const uiOverlayStyle = {
        position: 'fixed',
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        zIndex: 10000,
        pointerEvents: 'none',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        padding: '20px'
    };

    const popupBaseStyle = {
        pointerEvents: 'auto',
        backgroundColor: 'rgba(17, 24, 39, 0.95)',
        backdropFilter: 'blur(12px)',
        borderRadius: '24px',
        border: '1px solid rgba(255, 255, 255, 0.1)',
        boxShadow: '0 25px 50px -12px rgba(0, 0, 0, 0.5)',
        padding: '32px',
        maxWidth: '600px',
        width: '100%',
        margin: '0 auto'
    };

    const titleStyle = {
        fontSize: '36px',
        fontWeight: '900',
        color: '#ffffff',
        textAlign: 'center',
        marginBottom: '32px',
        letterSpacing: '-0.02em'
    };

    const buttonBaseStyle = {
        padding: '16px',
        borderRadius: '16px',
        border: '2px solid transparent',
        backgroundColor: 'rgba(31, 41, 55, 0.5)',
        cursor: 'pointer',
        transition: 'all 0.2s',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        position: 'relative',
        overflow: 'hidden'
    };

    return (
        <div style={{ position: 'fixed', top: 0, left: 0, right: 0, bottom: 0, backgroundColor: '#000000', overflow: 'hidden', fontFamily: 'system-ui, sans-serif' }}>
            {/* Game Canvas Layer */}
            <div 
                style={{ 
                    position: 'absolute',
                    top: 0,
                    left: 0,
                    right: 0,
                    bottom: 0,
                    filter: 'contrast(1.2) brightness(1.1) blur(0.4px) saturate(0.8)',
                    zIndex: 1
                }}
            >
                <canvas 
                    ref={canvasRef} 
                    style={{ 
                        display: 'block',
                        width: '100%',
                        height: '100%'
                    }} 
                />
            </div>

            {/* Bounding Boxes Layer - Above canvas, below popups */}
            {gameState === 'RACING' && (
                <div style={{
                    position: 'fixed',
                    top: 0,
                    left: 0,
                    right: 0,
                    bottom: 0,
                    zIndex: 5000,
                    pointerEvents: 'none'
                }}>
                    {racers.map(r => (
                        <RacerOutline key={r.id} racer={r} />
                    ))}
                </div>
            )}

            {/* UI Overlay Layer */}
            <div style={uiOverlayStyle}>
                {/* SETUP POPUP - Choose Your Fighter */}
                {gameState === 'SETUP' && (
                    <div style={popupBaseStyle}>
                        <h1 style={titleStyle}>MICRO DERBY</h1>
                        
                        {/* Difficulty Slider Section */}
                        <div style={{
                            backgroundColor: 'rgba(0, 0, 0, 0.4)',
                            padding: '24px',
                            borderRadius: '16px',
                            marginBottom: '32px',
                            border: '1px solid rgba(255, 255, 255, 0.05)'
                        }}>
                            <div style={{
                                display: 'flex',
                                justifyContent: 'space-between',
                                alignItems: 'center',
                                marginBottom: '16px',
                                color: '#d1d5db',
                                fontSize: '12px',
                                fontFamily: 'monospace',
                                textTransform: 'uppercase',
                                letterSpacing: '0.1em'
                            }}>
                                <span>Risk Level</span>
                                <span style={{ color: '#818cf8', fontWeight: 'bold' }}>TOP {difficulty} Finishers</span>
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
                                    height: '8px',
                                    borderRadius: '4px',
                                    backgroundColor: '#374151',
                                    outline: 'none',
                                    cursor: 'pointer',
                                    WebkitAppearance: 'none',
                                    appearance: 'none'
                                }}
                            />
                            <p style={{
                                textAlign: 'center',
                                color: '#6b7280',
                                fontSize: '12px',
                                marginTop: '12px',
                                marginBottom: 0
                            }}>
                                Race ends when {difficulty} racers reach the goal. You must be one of them.
                            </p>
                        </div>

                        {/* Fighter Selection Grid */}
                        <div style={{
                            display: 'grid',
                            gridTemplateColumns: 'repeat(4, 1fr)',
                            gap: '16px'
                        }}>
                            {HORSES_CONFIG.map(h => (
                                <button
                                    key={h.id}
                                    onClick={() => {
                                        setBet(h.id);
                                        setRacers(initializeRace(window.innerWidth, window.innerHeight));
                                        gameStateRef.current = 'RACING';
                                        setGameState('RACING');
                                    }}
                                    onMouseEnter={(e) => {
                                        e.currentTarget.style.transform = 'scale(1.05)';
                                        e.currentTarget.style.backgroundColor = 'rgba(55, 65, 81, 0.7)';
                                    }}
                                    onMouseLeave={(e) => {
                                        e.currentTarget.style.transform = 'scale(1)';
                                        e.currentTarget.style.backgroundColor = 'rgba(31, 41, 55, 0.5)';
                                    }}
                                    style={{
                                        ...buttonBaseStyle,
                                        borderColor: bet === h.id ? h.color : 'transparent'
                                    }}
                                >
                                    <span style={{ fontSize: '36px', marginBottom: '8px', filter: bet === h.id ? 'grayscale(0)' : 'grayscale(1)' }}>
                                        {h.emoji}
                                    </span>
                                    <div style={{
                                        position: 'absolute',
                                        bottom: 0,
                                        left: 0,
                                        right: 0,
                                        height: '4px',
                                        backgroundColor: h.color,
                                        opacity: 0.5
                                    }} />
                                </button>
                            ))}
                        </div>
                    </div>
                )}

                {/* RESULTS POPUP */}
                {gameState === 'FINISHED' && (
                    <div style={{
                        ...popupBaseStyle,
                        maxWidth: '500px',
                        textAlign: 'center'
                    }}>
                        <h2 style={{
                            fontSize: '28px',
                            fontWeight: '900',
                            color: '#ffffff',
                            marginBottom: '24px'
                        }}>
                            EXPERIMENT COMPLETE
                        </h2>
                        
                        {/* Results List */}
                        <div style={{
                            marginBottom: '32px',
                            textAlign: 'left',
                            backgroundColor: 'rgba(0, 0, 0, 0.3)',
                            padding: '16px',
                            borderRadius: '12px',
                            maxHeight: '240px',
                            overflowY: 'auto'
                        }}>
                            <h3 style={{
                                fontSize: '10px',
                                fontWeight: 'bold',
                                color: '#6b7280',
                                marginBottom: '12px',
                                textTransform: 'uppercase',
                                letterSpacing: '0.1em'
                            }}>
                                Qualified Specimens (Top {difficulty})
                            </h3>
                            <ol style={{ listStyle: 'none', padding: 0, margin: 0 }}>
                                {finishedRankings.map((rId, idx) => {
                                    const racer = HORSES_CONFIG.find(h => h.id === rId);
                                    const isMyBet = rId === bet;
                                    return (
                                        <li 
                                            key={rId} 
                                            style={{
                                                display: 'flex',
                                                alignItems: 'center',
                                                padding: '8px',
                                                borderRadius: '8px',
                                                marginBottom: '8px',
                                                backgroundColor: isMyBet ? 'rgba(99, 102, 241, 0.3)' : 'transparent',
                                                border: isMyBet ? '1px solid rgba(99, 102, 241, 0.5)' : 'none'
                                            }}
                                        >
                                            <span style={{ color: '#6b7280', fontFamily: 'monospace', width: '24px' }}>
                                                {idx + 1}.
                                            </span>
                                            <span style={{ marginRight: '8px', fontSize: '20px' }}>{racer.emoji}</span>
                                            <span style={{
                                                color: isMyBet ? '#ffffff' : '#d1d5db',
                                                fontWeight: isMyBet ? 'bold' : 'normal',
                                                flex: 1
                                            }}>
                                                {racer.name}
                                            </span>
                                            {isMyBet && (
                                                <span style={{
                                                    fontSize: '10px',
                                                    backgroundColor: '#6366f1',
                                                    color: '#ffffff',
                                                    padding: '4px 8px',
                                                    borderRadius: '12px',
                                                    marginLeft: 'auto'
                                                }}>
                                                    YOU
                                                </span>
                                            )}
                                        </li>
                                    );
                                })}
                            </ol>
                        </div>

                        {/* Status Message */}
                        <div style={{
                            fontSize: '18px',
                            fontWeight: 'bold',
                            marginBottom: '24px',
                            color: statusText.includes('DEFEAT') ? '#f87171' : '#4ade80'
                        }}>
                            {statusText}
                        </div>

                        {/* Restart Button */}
                        <button 
                            onClick={() => {
                                gameStateRef.current = 'SETUP';
                                setGameState('SETUP');
                            }}
                            onMouseEnter={(e) => {
                                e.currentTarget.style.backgroundColor = '#e5e7eb';
                            }}
                            onMouseLeave={(e) => {
                                e.currentTarget.style.backgroundColor = '#ffffff';
                            }}
                            style={{
                                width: '100%',
                                backgroundColor: '#ffffff',
                                color: '#000000',
                                padding: '16px 32px',
                                borderRadius: '12px',
                                fontWeight: 'bold',
                                fontSize: '14px',
                                letterSpacing: '0.05em',
                                border: 'none',
                                cursor: 'pointer',
                                transition: 'background-color 0.2s'
                            }}
                        >
                            RUN NEW TEST
                        </button>
                    </div>
                )}
            </div>
        </div>
    );
};

export default App;