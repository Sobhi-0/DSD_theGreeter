// src/components/Sidebar.jsx

import React from 'react';

const SideNavBar = () => {
    return (
        <div className="sidebar">
             <div className="header">
                <img src="src/assets/sphero-icon.png" alt="The Greeter Logo" className="logo" />
                <h2>The Greeter</h2>
            </div>
            <ul>
                <li><a href="./home">Home</a></li>
                <li><a href="./map">Map Editor</a></li>
                <li><a href="./devices">Devices</a></li>
                <li><a href="./contact">About</a></li>
            </ul>
            <div className="footer">
               <footer>&copy; The Greeter 2024-2025</footer>
            </div>
        </div>
    );
};

export default SideNavBar;
