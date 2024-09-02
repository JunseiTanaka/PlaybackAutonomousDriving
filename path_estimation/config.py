from dotenv import load_dotenv
import os

load_dotenv()

TOWARD_CSV_DIR_PATH = os.getenv('TOWARD_CSV_DIR_PATH')
INVED_CSV_DIR_PATH= os.getenv('INVED_CSV_DIR_PATH')
PLAYBACK_CSV_DIR_PATH = os.getenv('PLAYBACK_CSV_DIR_PATH')
BACKUP_CSV_DIR_PATH = os.getenv('BACKUP_CSV_DIR_PATH')

