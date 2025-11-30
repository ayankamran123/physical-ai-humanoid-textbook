import React, { useState } from 'react';
import styles from './LoginModal.module.css';

const LoginModal = ({ onClose }) => {
  const [isLogin, setIsLogin] = useState(true);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');

  const handleSubmit = async (e) => {
    e.preventDefault();
    const url = isLogin ? '/token' : '/signup';
    const body = isLogin
      ? JSON.stringify({ username: email, password })
      : JSON.stringify({ email, password, software_background: softwareBackground, hardware_background: hardwareBackground });

    const headers = {
        'Content-Type': isLogin ? 'application/x-www-form-urlencoded' : 'application/json',
    };

    if (isLogin) {
        const formData = new URLSearchParams();
        formData.append('username', email);
        formData.append('password', password);
        body = formData.toString();
    }


    try {
      const response = await fetch(`http://localhost:8000${url}`, {
        method: 'POST',
        headers: headers,
        body: body,
      });

      if (response.ok) {
        const data = await response.json();
        localStorage.setItem('token', data.access_token);
        onClose();
        window.location.reload();
      } else {
        console.error('Failed to authenticate');
      }
    } catch (error) {
      console.error('Error:', error);
    }
  };

  return (
    <div className={styles.modalOverlay}>
      <div className={styles.modal}>
        <button className={styles.closeButton} onClick={onClose}>&times;</button>
        <h2>{isLogin ? 'Login' : 'Sign Up'}</h2>
        <form onSubmit={handleSubmit}>
          <input
            type="email"
            placeholder="Email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
          />
          <input
            type="password"
            placeholder="Password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
          />
          {!isLogin && (
            <>
              <input
                type="text"
                placeholder="Software Development Background"
                value={softwareBackground}
                onChange={(e) => setSoftwareBackground(e.target.value)}
                required
              />
              <input
                type="text"
                placeholder="Hardware/IoT Background"
                value={hardwareBackground}
                onChange={(e) => setHardwareBackground(e.target.value)}
                required
              />
            </>
          )}
          <button type="submit">{isLogin ? 'Login' : 'Sign Up'}</button>
        </form>
        <p>
          {isLogin ? "Don't have an account?" : 'Already have an account?'}
          <button onClick={() => setIsLogin(!isLogin)}>
            {isLogin ? 'Sign Up' : 'Login'}
          </button>
        </p>
      </div>
    </div>
  );
};

export default LoginModal;
