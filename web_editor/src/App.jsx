import React from 'react';
import './App.css';
import SideNavBar from './components/SideNavBar';
import { Outlet } from 'react-router-dom';


const App = () => {
    return (
        <div>
            <SideNavBar />  {/* Sidebar siempre visible */}
            <main>
                <Outlet/>
            </main>
        </div>
    );
};

export default App;