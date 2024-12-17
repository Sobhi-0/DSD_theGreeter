import { createBrowserRouter, Navigate } from "react-router-dom";
import App from "../App"; // Main layout component
import Devices from "../components/DevicesList";
import Contact from "../pages/Contact"; // Contact component for /contact route
import Home from "../pages/Home"; // Home component for /home route
import MapEditor from "../pages/MapEditor"; // MapEditor component for /map route

const router = createBrowserRouter([
    {
        path: "/",
        element: <App />, // Main layout that includes Sidebar
        children: [
            {
                index: true, // This will make this route the default
                element: <Navigate to="home" /> // Redirect to /home
            },
            {
                path: "/home",
                element: <Home />, // Render Home component at /home
            },
            {
                path: "/map",
                element: <MapEditor />, // Render MapEditor at /map
            },
            {
                path: "/devices",
                element: <Devices />, // Render DevicesList at /devices
            },
            {
                path: "/contact",
                element: <Contact />, // Render Contact component at /contact
            },
        ],
    },
]);

export default router;
