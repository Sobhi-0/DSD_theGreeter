import { useEffect, useRef, useState } from 'react';
import axios from 'axios';

const TileMap = () => {
    const canvasRef = useRef(null);
    const [currentTool, setCurrentTool] = useState('none');
    const [devices, setDevices] = useState([]);
    const [draggedDevice, setDraggedDevice] = useState(null);
    const [isMouseDown, setIsMouseDown] = useState(false);
    const [grid, setGrid] = useState(Array(16).fill().map(() => Array(16).fill({ type: 'empty', data: null }))); // Grid with 'empty' cells
    const tileSize = 32;
    const gridWidth = 16;
    const gridHeight = 16;

    // Styles
    const buttonStyle = {
        backgroundColor: '#ffffff',
        color: '#000000',
        border: '1px solid #ccc',
        padding: '8px 16px',
        cursor: 'pointer',
        fontSize: '14px',
        borderRadius: '4px',
    };

    const activeButtonStyle = {
        backgroundColor: '#3182ce',
        color: '#ffffff',
        border: '1px solid #3182ce',
        padding: '8px 16px',
        cursor: 'pointer',
        fontSize: '14px',
        borderRadius: '4px',
    };

    const canvasStyle = {
        border: '1px solid #ccc',
        cursor: 'crosshair',
    };

    const deviceListContainerStyle = {
        width: '300px',
        padding: '16px',
        borderLeft: '1px solid #ccc',
        marginLeft: '16px', // Move the table a bit to the right
        height: '400px', // Set a fixed height for the container
        overflowY: 'auto', // Enable vertical scroll if content exceeds the container height
    };

    const tableStyle = {
        width: '100%',
        borderCollapse: 'collapse',
    };

    const tableHeaderStyle = {
        backgroundColor: '#f1f1f1',
        padding: '8px',
        textAlign: 'left',
        fontSize: '14px',
        fontWeight: 'bold',
    };

    const tableRowStyle = {
        cursor: 'pointer',
        transition: 'background-color 0.3s ease',
    };

    const tableCellStyle = {
        padding: '8px',
        borderBottom: '1px solid #ddd',
        fontSize: '14px',
    };

    // Fetch devices from the server (lights)
    useEffect(() => {
        const fetchDevices = async () => {
            try {
                const response = await axios.get('http://localhost:5000/devices/lights');
                const uniqueDevices = [];
                const seenEntityIds = new Set();
                response.data.forEach(device => {
                    if (!seenEntityIds.has(device.entity_id)) {
                        uniqueDevices.push(device);
                        seenEntityIds.add(device.entity_id);
                    }
                });
                setDevices(uniqueDevices); // Store unique lights
            } catch (error) {
                console.error("Error fetching devices:", error);
            }
        };
        fetchDevices();
    }, []);

    // Function to generate random colors for lights
    const getRandomColor = () => {
        const letters = '0123456789ABCDEF';
        let color = '#';
        for (let i = 0; i < 6; i++) {
            color += letters[Math.floor(Math.random() * 16)];
        }
        return color;
    };

    // Draw grid and devices (lights)
    const drawGrid = (context) => {
        const canvas = context.canvas;
        context.clearRect(0, 0, canvas.width, canvas.height);

        // Draw grid cells
        grid.forEach((row, y) => {
            row.forEach((cell, x) => {
                context.fillStyle = '#f0f0f0'; // Light background color for grid
                context.fillRect(x * tileSize, y * tileSize, tileSize, tileSize);

                // Draw walls
                if (cell.type === 'wall') {
                    context.fillStyle = '#808080'; // Wall color
                    context.fillRect(x * tileSize, y * tileSize, tileSize, tileSize);
                }

                // Draw furniture
                if (cell.type === 'furniture') {
                    context.fillStyle = '#8B4513'; // Furniture color
                    context.fillRect(x * tileSize + 4, y * tileSize + 4, tileSize - 8, tileSize - 8); // Draw furniture with padding
                }

                // Draw devices (lights)
                if (cell.type === 'light' && cell.data) {
                    drawDevice(context, x, y, cell.data.color);
                }

                // Tile border
                context.strokeStyle = '#cbd5e0';
                context.lineWidth = 1;
                context.strokeRect(x * tileSize, y * tileSize, tileSize, tileSize);
            });
        });
    };

    // Draw a device (light) on the canvas
    const drawDevice = (context, x, y, color) => {
        context.fillStyle = color;
        context.beginPath();
        context.arc(x * tileSize + tileSize / 2, y * tileSize + tileSize / 2, tileSize / 4, 0, Math.PI * 2);
        context.fill();
    };

    // Handle mouse interactions on the canvas
    const handleCanvasInteraction = (e) => {
        const canvas = canvasRef.current;
        const rect = canvas.getBoundingClientRect();
        const x = Math.floor((e.clientX - rect.left) / tileSize);
        const y = Math.floor((e.clientY - rect.top) / tileSize);

        if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight) {
            const newGrid = [...grid];

            switch (currentTool) {
                case 'erase':
                    newGrid[y][x] = { type: 'empty', data: null }; // Erase cell
                    break;
                case 'wall':
                    newGrid[y][x] = { type: 'wall', data: null }; // Place wall
                    break;
                case 'furniture':
                    newGrid[y][x] = { type: 'furniture', data: null }; // Place furniture
                    break;
                case 'place':
                    if (newGrid[y][x].type === 'empty') {
                        newGrid[y][x] = { type: 'light', data: { ...draggedDevice, color: getRandomColor() } }; // Place light with random color
                    }
                    break;
                default:
                    break;
            }
            setGrid(newGrid);
        }
    };

    // Handle mouse events for dragging and placing devices
    const handleMouseDown = (e) => {
        setIsMouseDown(true);
        handleCanvasInteraction(e); // Place object immediately
    };

    const handleMouseUp = () => {
        setIsMouseDown(false);
    };

    const handleMouseMove = (e) => {
        if (isMouseDown) {
            handleCanvasInteraction(e); // Keep placing devices while the mouse is down
        }
    };

    // Handle drag events for devices (lights)
    const handleDragStart = (device) => {
        setDraggedDevice(device);
    };

    const handleDragOver = (e) => {
        e.preventDefault(); // Allow dragging over canvas
    };

    const handleDragEnd = () => {
        setDraggedDevice(null);
    };

    // Draw grid and devices
    useEffect(() => {
        const canvas = canvasRef.current;
        if (canvas) {
            const context = canvas.getContext('2d');
            drawGrid(context);
        }
    }, [grid]); // Redraw grid when it changes

    return (
        <div style={{ display: 'flex', justifyContent: 'space-between', padding: '16px' }}>
            {/* Left side with the map */}
            <div>
                {/* Tools for drawing and placing objects */}
                <div style={{ display: 'flex', gap: '8px', marginBottom: '16px', flexWrap: 'wrap', justifyContent: 'center' }}>

                    <button
                        onClick={() => setCurrentTool(currentTool === 'wall' ? 'none' : 'wall')}
                        style={currentTool === 'wall' ? activeButtonStyle : buttonStyle}
                    >
                        Draw Wall: {currentTool === 'wall' ? 'On' : 'Off'}
                    </button>

                    <button
                        onClick={() => setCurrentTool(currentTool === 'furniture' ? 'none' : 'furniture')}
                        style={currentTool === 'furniture' ? activeButtonStyle : buttonStyle}
                    >
                        Draw Furniture: {currentTool === 'furniture' ? 'On' : 'Off'}
                    </button>

                    <button
                        onClick={() => setCurrentTool(currentTool === 'place' ? 'none' : 'place')}
                        style={currentTool === 'place' ? activeButtonStyle : buttonStyle}
                    >
                        Place Light: {currentTool === 'place' ? 'On' : 'Off'}
                    </button>

                    <button
                        onClick={() => setCurrentTool(currentTool === 'erase' ? 'none' : 'erase')}
                        style={currentTool === 'erase' ? activeButtonStyle : buttonStyle}
                    >
                        Erase: {currentTool === 'erase' ? 'On' : 'Off'}
                    </button>
                </div>

                <canvas
                    ref={canvasRef}
                    width={gridWidth * tileSize}
                    height={gridHeight * tileSize}
                    style={canvasStyle}
                    onMouseDown={handleMouseDown}
                    onMouseUp={handleMouseUp}
                    onMouseMove={handleMouseMove}
                    onDragOver={handleDragOver}
                    onDragEnd={handleDragEnd}
                />
            </div>

            {/* Right side with device list */}
            <div style={deviceListContainerStyle}>
                <h3>Devices (Lights)</h3>
                <table style={tableStyle}>
                    <thead>
                        <tr>
                            <th style={tableHeaderStyle}>Name</th>
                            <th style={tableHeaderStyle}>State</th>
                        </tr>
                    </thead>
                    <tbody>
                        {devices.map((device, index) => (
                            <tr
                                key={index}
                                style={tableRowStyle}
                                draggable
                                onDragStart={() => handleDragStart(device)}
                            >
                                <td style={tableCellStyle}>{device.friendly_name || device.name}</td>
                                <td style={tableCellStyle}>{device.state}</td>
                            </tr>
                        ))}
                    </tbody>
                </table>
            </div>
        </div>
    );
};

export default TileMap;
