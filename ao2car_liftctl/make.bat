pyinstaller --onefile --exclude-module PySide2 --exclude-module PySide6 --add-data "config;config" app_windows.py
    
xcopy /E /I /Y config dist\config