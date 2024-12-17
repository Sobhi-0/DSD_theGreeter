import React, { useEffect, useState } from 'react';
import '../css/Home.css';

const Home = () => {
    const [welcomeText] = useState('WELCOME TO THE GREETER');

    useEffect(() => {
        // Add the class to body when on Home page
        document.body.classList.add('home-page');

        // Cleanup: remove the class when leaving the Home page
        return () => {
            document.body.classList.remove('home-page');
        };
    }, []);

    return (
        <div className="home-container">
            <div className="home-overlay">
                <h1 className="home-heading">{welcomeText}</h1>
                <p className="home-paragraph">
                    Workng on page...
                </p>
            </div>
        </div>
    );
};

export default Home;
