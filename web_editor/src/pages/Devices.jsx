import React from 'react';
import '../css/devices.css';

const DeviceCard = ({ device }) => {
  return (
    <div className="device-card">
      <h3>{device.friendly_name}</h3>
      <p><strong>Entity ID:</strong> {device.entity_id}</p>
      <p><strong>State:</strong> {device.state}</p>
    </div>
  );
};

export default DeviceCard;