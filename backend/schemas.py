from pydantic import BaseModel

class UserCreate(BaseModel):
    email: str
    password: str
    software_background: str
    hardware_background: str

class User(BaseModel):
    id: int
    email: str
    software_background: str
    hardware_background: str

    class Config:
        orm_mode = True

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    email: str | None = None

class PersonalizeRequest(BaseModel):
    current_page_text: str

class TranslateRequest(BaseModel):
    current_page_text: str
