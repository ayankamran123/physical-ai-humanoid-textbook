
import os
import glob
from dotenv import load_dotenv
from pathlib import Path

env_path = Path(__file__).parent / '.env'
load_dotenv(dotenv_path=env_path)

import google.generativeai as genai
from qdrant_client import QdrantClient, models

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "").strip().replace('"', '')
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY not found in environment variables.")
if not QDRANT_URL:
    raise ValueError("QDRANT_URL not found in environment variables.")
if not QDRANT_API_KEY:
    raise ValueError("QDRANT_API_KEY not found in environment variables.")

print(f"Gemini Key found: {GEMINI_API_KEY[:5]}...")
genai.configure(api_key=GEMINI_API_KEY)
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    timeout=60
)

COLLECTION_NAME = "book_knowledge"
DOCS_PATH = "../docs"

def get_embedding(text):
    response = genai.embed_content(
        model="models/text-embedding-004",
        content=text
    )
    return response['embedding']

def chunk_text(text, chunk_size=1000, overlap=200):
    chunks = []
    for i in range(0, len(text), chunk_size - overlap):
        chunks.append(text[i:i + chunk_size])
    return chunks

def ingest_documents():
    if not qdrant_client.collection_exists(collection_name=COLLECTION_NAME):
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
        )

    documents = []
    for md_file in glob.glob(os.path.join(DOCS_PATH, "*.md")):
        with open(md_file, "r") as f:
            content = f.read()
            chunks = chunk_text(content)
            for chunk in chunks:
                documents.append({"text": chunk, "source": os.path.basename(md_file)})

    points = []
    for i, doc in enumerate(documents):
        embedding = get_embedding(doc["text"])
        points.append(
            models.PointStruct(
                id=i,
                vector=embedding,
                payload={"text": doc["text"], "source": doc["source"]},
            )
        )

    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        points=points,
        wait=True,
    )
    print(f"Ingested {len(documents)} chunks into Qdrant collection {COLLECTION_NAME}")

if __name__ == "__main__":
    ingest_documents()
