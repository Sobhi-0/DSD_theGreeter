import { useEffect, useRef, useState } from 'react';

const TileMap = () => {
    const canvasRef = useRef(null);
    const [currentTool, setCurrentTool] = useState('none'); // 'none', 'draw', 'erase', 'special'
    const [isDragging, setIsDragging] = useState(false);

    // Grid configuration
    const tileSize = 32;
    const gridWidth = 16;
    const gridHeight = 16;

    // Initialize grid data with objects instead of simple values
    const [grid, setGrid] = useState(
        Array(gridHeight).fill().map(() =>
            Array(gridWidth).fill().map(() => ({
                type: 'empty',
                data: {}
            }))
        )
    );

    // Draw special tile
    const drawSpecialTile = (context, x, y) => {
        // Semi-transparent blue background
        context.fillStyle = 'rgba(0, 100, 255, 0.3)';
        context.fillRect(x * tileSize, y * tileSize, tileSize, tileSize);

        // Add a border
        context.strokeStyle = 'rgba(0, 100, 255, 0.8)';
        context.lineWidth = 2;
        context.strokeRect(x * tileSize + 1, y * tileSize + 1, tileSize - 2, tileSize - 2);

        // Add a small icon or marker
        context.fillStyle = 'rgba(0, 100, 255, 0.8)';
        const centerX = x * tileSize + tileSize / 2;
        const centerY = y * tileSize + tileSize / 2;
        const radius = 4;
        context.beginPath();
        context.arc(centerX, centerY, radius, 0, Math.PI * 2);
        context.fill();
    };

    // Draw striped tile
    const drawStripedTile = (context, x, y) => {
        // Fill base orange color
        context.fillStyle = '#FF9933';
        context.fillRect(x * tileSize, y * tileSize, tileSize, tileSize);

        // Draw white stripes
        context.fillStyle = 'white';
        const stripeWidth = 4;
        const numStripes = 3;
        const gap = (tileSize - (numStripes * stripeWidth)) / (numStripes + 1);

        for (let i = 0; i < numStripes; i++) {
            const stripeX = x * tileSize + gap + i * (stripeWidth + gap);
            const stripeY = y * tileSize;
            context.fillRect(stripeX, stripeY, stripeWidth, tileSize);
        }
    };

    // Draw the grid
    const drawGrid = (context) => {
        const canvas = context.canvas;

        // Clear canvas
        context.clearRect(0, 0, canvas.width, canvas.height);

        // Draw filled cells
        grid.forEach((row, y) => {
            row.forEach((cell, x) => {
                if (cell.type === 'normal') {
                    drawStripedTile(context, x, y);
                } else if (cell.type === 'special') {
                    drawSpecialTile(context, x, y);
                }
            });
        });

        // Draw grid lines
        context.strokeStyle = '#cbd5e0';
        context.lineWidth = 1;

        // Vertical lines
        for (let x = 0; x <= canvas.width; x += tileSize) {
            context.beginPath();
            context.moveTo(x, 0);
            context.lineTo(x, canvas.height);
            context.stroke();
        }

        // Horizontal lines
        for (let y = 0; y <= canvas.height; y += tileSize) {
            context.beginPath();
            context.moveTo(0, y);
            context.lineTo(canvas.width, y);
            context.stroke();
        }
    };

    // Handle canvas click/drag
    const handleCanvasClick = (e) => {
        if (currentTool === 'none') return;

        const canvas = canvasRef.current;
        const rect = canvas.getBoundingClientRect();
        const x = Math.floor((e.clientX - rect.left) / tileSize);
        const y = Math.floor((e.clientY - rect.top) / tileSize);

        if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight) {
            const newGrid = [...grid];
            if (currentTool === 'erase') {
                newGrid[y][x] = { type: 'empty', data: {} };
            } else if (currentTool === 'special') {
                newGrid[y][x] = {
                    type: 'special',
                    data: {
                        id: `special-${Date.now()}`,
                        createdAt: new Date().toISOString(),
                        customValue: 0
                    }
                };
            } else if (currentTool === 'draw') {
                newGrid[y][x] = { type: 'normal', data: {} };
            }
            setGrid(newGrid);
        }
    };

    // Initialize canvas and draw initial grid
    useEffect(() => {
        const canvas = canvasRef.current;
        const context = canvas.getContext('2d');
        drawGrid(context);
    }, [grid]);

    const buttonStyle = {
        padding: '8px 16px',
        fontSize: '14px',
        borderRadius: '4px',
        border: 'none',
        cursor: 'pointer',
        backgroundColor: '#718096',
        color: 'white',
        margin: '8px'
    };

    const activeButtonStyle = {
        ...buttonStyle,
        backgroundColor: '#68D391'
    };

    return (
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', padding: '16px' }}>
            <div style={{ display: 'flex', gap: '8px', marginBottom: '16px', flexWrap: 'wrap', justifyContent: 'center' }}>
                <button
                    onClick={() => setCurrentTool(currentTool === 'draw' ? 'none' : 'draw')}
                    style={currentTool === 'draw' ? activeButtonStyle : buttonStyle}
                >
                    Draw: {currentTool === 'draw' ? 'On' : 'Off'}
                </button>

                <button
                    onClick={() => setCurrentTool(currentTool === 'erase' ? 'none' : 'erase')}
                    style={currentTool === 'erase' ? activeButtonStyle : buttonStyle}
                >
                    Erase: {currentTool === 'erase' ? 'On' : 'Off'}
                </button>

                <button
                    onClick={() => setCurrentTool(currentTool === 'special' ? 'none' : 'special')}
                    style={currentTool === 'special' ? activeButtonStyle : buttonStyle}
                >
                    Special: {currentTool === 'special' ? 'On' : 'Off'}
                </button>

            </div>

            <canvas
                ref={canvasRef}
                width={gridWidth * tileSize}
                height={gridHeight * tileSize}
                style={{ border: '1px solid #CBD5E0' }}
                onClick={handleCanvasClick}
                onMouseDown={() => setIsDragging(true)}
                onMouseUp={() => setIsDragging(false)}
                onMouseMove={(e) => isDragging && handleCanvasClick(e)}
            />
        </div>
    );
};

export default TileMap;
