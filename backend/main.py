import os

import google.generativeai as genai
from dotenv import load_dotenv
from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import OAuth2PasswordRequestForm
from pydantic import BaseModel
from qdrant_client import QdrantClient
from sqlalchemy.orm import Session

import auth, models, schemas
from database import SessionLocal, engine, get_db

# Create database tables
models.Base.metadata.create_all(bind=engine)

# Load environment variables
load_dotenv()

# Configure Gemini API
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY not found in environment variables.")
genai.configure(api_key=GEMINI_API_KEY)

# Configure Qdrant client
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
if not QDRANT_URL or not QDRANT_API_KEY:
    raise ValueError("QDRANT_URL or QDRANT_API_KEY not found in environment variables.")

qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    timeout=60,
)

COLLECTION_NAME = "book_knowledge"

app = FastAPI()

# Configure CORS
origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.post("/signup", response_model=schemas.User)
def create_user(user: schemas.UserCreate, db: Session = Depends(get_db)):
    db_user = db.query(models.User).filter(models.User.email == user.email).first()
    if db_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    hashed_password = auth.get_password_hash(user.password)
    db_user = models.User(
        email=user.email,
        hashed_password=hashed_password,
        software_background=user.software_background,
        hardware_background=user.hardware_background,
    )
    db.add(db_user)
    db.commit()
    db.refresh(db_user)
    return db_user


@app.post("/token", response_model=schemas.Token)
def login_for_access_token(
    form_data: OAuth2PasswordRequestForm = Depends(), db: Session = Depends(get_db)
):
    user = db.query(models.User).filter(models.User.email == form_data.username).first()
    if not user or not auth.verify_password(form_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    access_token = auth.create_access_token(data={"sub": user.email})
    return {"access_token": access_token, "token_type": "bearer"}


@app.post("/api/personalize")
def personalize_content(
    request: schemas.PersonalizeRequest,
    current_user: models.User = Depends(auth.get_current_user),
):
    model = genai.GenerativeModel("models/gemini-flash-latest")
    prompt = f"Rewrite this text for someone with a {current_user.software_background} software background and a {current_user.hardware_background} hardware background. Use relevant analogies.\n\nText: {request.current_page_text}"
    response = model.generate_content(prompt)
    return {"personalized_text": response.text}


@app.post("/api/translate")
def translate_content(request: schemas.TranslateRequest):
    model = genai.GenerativeModel("models/gemini-flash-latest")
    prompt = f"Translate this technical content to Urdu.\n\nText: {request.current_page_text}"
    response = model.generate_content(prompt)
    return {"translated_text": response.text}


class ChatRequest(BaseModel):
    message: str
    selectedText: str = None


@app.post("/chat")
async def chat(request: ChatRequest):
    try:
        query_embedding_response = genai.embed_content(
            model="models/text-embedding-004", content=request.message
        )
        query_embedding = query_embedding_response["embedding"]

        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=3,
        )

        context = ""
        for hit in search_result:
            context += hit.payload["text"] + "\n"

        if request.selectedText:
            context += "\n\nUser selected text: " + request.selectedText

        model = genai.GenerativeModel("models/gemini-flash-latest")

        chat_session = model.start_chat(history=[])

        prompt = f"""
You are a helpful AI assistant for a technical book.
Below is some context from the book:
{context}

Instructions:
- If the user's input is a greeting or general conversation (e.g., "Hi", "My name is...", "Who are you?"), answer naturally and politely without needing the context.
- If the user asks a technical question about the book, use the provided context to answer.
- Only say "I don't have enough information" if the user asks a specific technical question that is NOT covered in the context.

Question: {request.message}
"""

        response = chat_session.send_message(prompt)
        return {"answer": response.text}
    except Exception as e:
        print(f"Error in /chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"An error occurred: {e}")


@app.get("/")
async def root():
    return {"message": "FastAPI RAG Backend is running!"}
