import React, { useState } from 'react';
import styles from './LoginModal.module.css';

const LoginModal = ({ onClose, onLoginSuccess }) => {
  const [isLogin, setIsLogin] = useState(true);
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');
  const [error, setError] = useState('');

  const handleLogin = async (e) => {
    e.preventDefault();
    setError('');
    try {
      const response = await fetch('http://localhost:8000/token', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: new URLSearchParams({
          username,
          password,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        localStorage.setItem('token', data.access_token);
        onLoginSuccess(); // Call the new prop
        // onClose(); // Remove page reload, as Root.js will handle re-render
        // window.location.reload();
      } else {
        const errorData = await response.json();
        setError(errorData.detail || 'Login failed');
      }
    } catch (err) {
      setError('An error occurred. Please try again.');
    }
  };

  const handleSignup = async (e) => {
    e.preventDefault();
    setError('');
    try {
      const response = await fetch('http://localhost:8000/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email: username, // Changed from 'username' to 'email'
          password: password,
          software_background: softwareBackground, // Moved from profile
          hardware_background: hardwareBackground, // Moved from profile
        }),
      });

      if (response.ok) {
        // After signup, automatically login is handled by calling onLoginSuccess directly.
        onLoginSuccess(); // Call the new prop
        // onClose(); // Remove page reload, as Root.js will handle re-render
        // window.location.reload();
      }
    } catch (err) {
      setError('An error occurred. Please try again.');
    }
  };

  return (
    <div className={styles.modalBackdrop}>
      <div className={styles.modalContent}>
        <button className={styles.closeButton} onClick={onClose}>&times;</button>
        <h2>{isLogin ? 'Login' : 'Sign Up'}</h2>
        <form onSubmit={isLogin ? handleLogin : handleSignup}>
          {error && <p className={styles.error}>{error}</p>}
          <div className={styles.formGroup}>
            <label htmlFor="username">Username</label>
            <input
              type="text"
              id="username"
              value={username}
              onChange={(e) => setUsername(e.target.value)}
              required
            />
          </div>
          <div className={styles.formGroup}>
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
            />
          </div>
          {!isLogin && (
            <>
              <div className={styles.formGroup}>
                <label htmlFor="softwareBackground">Software Background</label>
                <textarea
                  id="softwareBackground"
                  value={softwareBackground}
                  onChange={(e) => setSoftwareBackground(e.target.value)}
                  placeholder="e.g., Python, JavaScript, C++"
                />
              </div>
              <div className={styles.formGroup}>
                <label htmlFor="hardwareBackground">Hardware Background</label>
                <textarea
                  id="hardwareBackground"
                  value={hardwareBackground}
                  onChange={(e) => setHardwareBackground(e.target.value)}
                  placeholder="e.g., Arduino, Raspberry Pi, FPGAs"
                />
              </div>
            </>
          )}
          <button type="submit" className={styles.submitButton}>
            {isLogin ? 'Login' : 'Sign Up'}
          </button>
        </form>
        <p className={styles.toggleText}>
          {isLogin ? "Don't have an account?" : 'Already have an account?'}
          <button className={styles.toggleButton} onClick={() => setIsLogin(!isLogin)}>
            {isLogin ? 'Sign Up' : 'Login'}
          </button>
        </p>
      </div>
    </div>
  );
};

export default LoginModal;
