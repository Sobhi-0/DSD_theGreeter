import React, { useState, useEffect } from 'react';
import DeviceCard from '../pages/Devices';
import '../css/devices.css';
import { Bar, Pie } from 'react-chartjs-2';
import { Chart as ChartJS, CategoryScale, LinearScale, BarElement, Title, Tooltip, Legend, ArcElement } from 'chart.js';

// Registrar los elementos necesarios de Chart.js
ChartJS.register(CategoryScale, LinearScale, BarElement, Title, Tooltip, Legend, ArcElement);

const Devices = () => {
  const [devices, setDevices] = useState([]);
  const [filteredDevices, setFilteredDevices] = useState([]);
  const [selectedState, setSelectedState] = useState('all');
  const [stateCounts, setStateCounts] = useState({ on: 0, off: 0, idle: 0 });

  useEffect(() => {
    const fetchDevices = async () => {
      try {
        const response = await fetch('http://localhost:5000/devices/count'); 
        const data = await response.json();
        setStateCounts(data.state_counts);

        const uniqueDevices = Array.from(new Map(data.devices_info.map(device => [device.entity_id, device])).values());
        setDevices(uniqueDevices);
        setFilteredDevices(uniqueDevices);
      } catch (error) {
        console.error('Error fetching devices:', error);
      }
    };
    fetchDevices();
  }, []);

  const filterDevices = (state) => {
    if (state === 'all') {
      setFilteredDevices(devices);
    } else {
      setFilteredDevices(devices.filter(device => device.state === state));
    }
  };

  const handleFilterChange = (e) => {
    setSelectedState(e.target.value);
    filterDevices(e.target.value);
  };

  // Data para los gráficos
  const barData = {
    labels: ['On', 'Off', 'Idle'],
    datasets: [{
      label: 'Device Count by State',
      data: [stateCounts.on, stateCounts.off, stateCounts.idle],
      backgroundColor: ['#4db8b8', '#ff5733', '#f39c12'],
    }]
  };

  const pieData = {
    labels: ['On', 'Off', 'Idle'],
    datasets: [{
      label: 'Device State Distribution',
      data: [stateCounts.on, stateCounts.off, stateCounts.idle],
      backgroundColor: ['#4db8b8', '#ff5733', '#f39c12'],
    }]
  };

  return (
    <div className="devices-container">
      <h1>Devices</h1>

      <div className="filter-container">
        <label>Filter by state: </label>
        <select value={selectedState} onChange={handleFilterChange} className="filter-select">
          <option value="all">All ({stateCounts.on + stateCounts.off + stateCounts.idle})</option>
          <option value="on">On ({stateCounts.on})</option>
          <option value="off">Off ({stateCounts.off})</option>
          <option value="idle">Idle ({stateCounts.idle})</option>
        </select>
      </div>

      <div className="main-content">
        {/* Tabla de dispositivos */}
        <div className="devices-table">
          <table>
            <thead>
              <tr>
                <th>Entity ID</th>
                <th>Friendly Name</th>
                <th>State</th>
              </tr>
            </thead>
            <tbody>
              {filteredDevices.map((device) => (
                <tr key={device.entity_id}>
                  <td>{device.entity_id}</td>
                  <td>{device.friendly_name}</td>
                  <td>{device.state}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>

        {/* Contenedor de gráficos */}
        <div className="charts-container">
          <div className="chart">
            <Bar data={barData} />
          </div>
          <div className="chart pie">
            <Pie data={pieData} />
          </div>
        </div>
      </div>
    </div>
  );
};

export default Devices;
