import React, { useState, useRef, useEffect } from "react";
import styles from "./AIChatbot.module.css";

const AIChatbot = () => {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState([]);
    const [inputMessage, setInputMessage] = useState("");
    const messagesEndRef = useRef(null);

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    };

    useEffect(scrollToBottom, [messages]);

    const toggleChat = () => {
        setIsOpen(!isOpen);
    };

    const handleSendMessage = async () => {
        if (inputMessage.trim() === "") return;

        const userMessage = { text: inputMessage, sender: "user" };
        setMessages((prevMessages) => [...prevMessages, userMessage]);
        setInputMessage("");

        try {
            const selectedText = window.getSelection().toString();
            const response = await fetch("http://127.0.0.1:8000/chat", {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                },
                body: JSON.stringify({ message: inputMessage, selectedText }),
            });
            const data = await response.json();
            const aiMessage = { text: data.answer, sender: "ai" };
            setMessages((prevMessages) => [...prevMessages, aiMessage]);
        } catch (error) {
            console.error("Error sending message:", error);
            setMessages((prevMessages) => [
                ...prevMessages,
                {
                    text: "Error: Could not connect to the AI. Please try again later.",
                    sender: "ai",
                },
            ]);
        }
    };

    const handleKeyPress = (e) => {
        if (e.key === "Enter") {
            handleSendMessage();
        }
    };

    return (
        <div className={styles.chatContainer}>
            <button className={styles.chatButton} onClick={toggleChat}>
                Chat with AI
            </button>

            {isOpen && (
                <div className={styles.chatWindow}>
                    <div className={styles.chatHeader}>
                        AI Chatbot
                        <button
                            className={styles.closeButton}
                            onClick={toggleChat}
                        >
                            X
                        </button>
                    </div>
                    <div className={styles.chatMessages}>
                        {messages.map((msg, index) => (
                            <div
                                key={index}
                                className={`${styles.message} ${styles[msg.sender]}`}
                            >
                                {msg.text}
                            </div>
                        ))}
                        <div ref={messagesEndRef} />
                    </div>
                    <div className={styles.chatInputContainer}>
                        <input
                            type="text"
                            className={styles.chatInput}
                            placeholder="Type your message..."
                            value={inputMessage}
                            onChange={(e) => setInputMessage(e.target.value)}
                            onKeyPress={handleKeyPress}
                        />
                        <button
                            className={styles.sendButton}
                            onClick={handleSendMessage}
                        >
                            Send
                        </button>
                    </div>
                </div>
            )}
        </div>
    );
};

export default AIChatbot;
