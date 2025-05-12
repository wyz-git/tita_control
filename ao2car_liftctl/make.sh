# 打包 main.py
pyinstaller --onefile \
    --exclude-module PySide2 \
    --exclude-module PySide6 \
    --add-data "config:config" \
    main.py

# 打包 app.py
pyinstaller --onefile \
    --exclude-module PySide2 \
    --exclude-module PySide6 \
    --add-data "config:config" \
    app.py
    
cp -r config dist
