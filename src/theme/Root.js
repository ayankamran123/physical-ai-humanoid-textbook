import React, { useState, useEffect } from 'react';
import LoginModal from '@site/src/components/LoginModal'; // Use Docusaurus alias

export default function Root({ children }) {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    const token = localStorage.getItem('token');
    if (token) {
      // In a real application, you would validate the token here.
      // For now, we assume its existence means authentication.
      setIsAuthenticated(true);
    }
    setIsLoading(false);
  }, []);

  const handleLoginSuccess = () => {
    setIsAuthenticated(true);
    // No need to reload the page, React will re-render.
  };

  // If still loading, render nothing or a spinner to avoid flickering the modal
  if (isLoading) {
    return null; // Or return a loading spinner component
  }

  return (
    <div>
      {isAuthenticated ? (
        children // Render the actual app content if authenticated
      ) : (
        <LoginModal
          onLoginSuccess={handleLoginSuccess}
          // We don't need onClose for a mandatory gatekeeper, but if it were
          // to be closable, it should also trigger re-authentication check
          // or prevent closing.
          // For now, we'll pass a dummy onClose or omit it if LoginModal doesn't use it.
          // Let's assume LoginModal's onClose is not used for this gatekeeper scenario.
          // If it does, we might need to adjust its behavior or remove the close button.
        />
      )}
    </div>
  );
}
