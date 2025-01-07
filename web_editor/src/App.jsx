import React from 'react';
import './css/App.css';
import SideNavBar from './components/SideNavBar';
import { Outlet } from 'react-router-dom';


const App = () => {
    return (
        <div>
            <SideNavBar />  {/*Allways visible */}
            <main>
                <Outlet/>
            </main>
        </div>
    );
};

export default App;