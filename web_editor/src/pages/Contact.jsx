import React, { useEffect, useState } from 'react';
import '../css/Contact.css'; // Import the Contact-specific CSS

const Contact = () => {
    const [email, setEmail] = useState('contact@greeter.com');
    const [phone, setPhone] = useState('+1-234-567-890');

    useEffect(() => {
        // Add the class to body when on Contact page
        document.body.classList.add('contact-page');

        // Cleanup: remove the class when leaving the Contact page
        return () => {
            document.body.classList.remove('contact-page');
        };
    }, []);

    return (
        <div className="contact-container">
            <div className="contact-overlay">
                <h1 className="contact-heading">About US</h1>
                <p className="contact-paragraph">
                    If you have any questions or would like to get in touch, feel free to reach out to us.
                </p>
                <p className="contact-paragraph">
                    Email: <a href={`mailto:${email}`}>{email}</a>
                </p>
                <p className="contact-paragraph">
                    Phone: <a href={`tel:${phone}`}>{phone}</a>
                </p>
            </div>
        </div>
    );
};

export default Contact;
