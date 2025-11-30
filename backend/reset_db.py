
from database import engine
from models import Base
import models

print("--- STARTING DATABASE RESET ---")
# This deletes the OLD table that is missing columns
Base.metadata.drop_all(bind=engine)
print("Old tables dropped.")

# This creates the NEW table with 'email', 'software_background', etc.
Base.metadata.create_all(bind=engine)
print("New tables created successfully.")
print("--- RESET COMPLETE ---")
