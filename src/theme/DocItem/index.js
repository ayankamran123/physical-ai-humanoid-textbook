import React, { useState, useEffect } from 'react';
import OriginalDocItem from '@theme-original/DocItem';
import LoginModal from '@site/src/components/LoginModal';

export default function DocItem(props) {
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [showLoginModal, setShowLoginModal] = useState(false);
  const [pageText, setPageText] = useState('');
  const [personalizedText, setPersonalizedText] = useState('');
  const [translatedText, setTranslatedText] = useState('');

  useEffect(() => {
    const token = localStorage.getItem('token');
    if (token) {
      setIsLoggedIn(true);
    }
    // Extract text from the page
    const content = props.content;
    const text = content.toString(); // This is a placeholder. We need a better way to get the text.
    setPageText(text);
  }, [props.content]);

  const handlePersonalize = async () => {
    const token = localStorage.getItem('token');
    try {
      const response = await fetch('http://localhost:8000/api/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({ current_page_text: pageText }),
      });
      if (response.ok) {
        const data = await response.json();
        setPersonalizedText(data.personalized_text);
      }
    } catch (error) {
      console.error('Error personalizing content:', error);
    }
  };

  const handleTranslate = async () => {
    try {
      const response = await fetch('http://localhost:8000/api/translate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ current_page_text: pageText }),
      });
      if (response.ok) {
        const data = await response.json();
        setTranslatedText(data.translated_text);
      }
    } catch (error) {
      console.error('Error translating content:', error);
    }
  };

  const handleLogin = () => {
    setShowLoginModal(true);
  };

  const handleLogout = () => {
    localStorage.removeItem('token');
    setIsLoggedIn(false);
    window.location.reload();
  };


  return (
    <div>
      <div style={{ marginBottom: '1rem' }}>
        {isLoggedIn ? (
          <>
            <button onClick={handlePersonalize}>✨ Personalize</button>
            <button onClick={handleLogout}>Logout</button>
          </>
        ) : (
          <button onClick={handleLogin}>Login to Personalize</button>
        )}
        <button onClick={handleTranslate}>🌐 Translate to Urdu</button>
      </div>
      {showLoginModal && <LoginModal onClose={() => setShowLoginModal(false)} />}
      {personalizedText ? (
        <div dangerouslySetInnerHTML={{ __html: personalizedText }} />
      ) : translatedText ? (
        <div dangerouslySetInnerHTML={{ __html: translatedText }} />
      ) : (
        <OriginalDocItem {...props} />
      )}
    </div>
  );
}
