プロジェクト構成

.                                       # プロジェクトルート
├── config/                             # 定数
│   ├── __init__.py
│   ├── constant.py                     # 
│   └── state.py
│
├── data/                               # データベース関連
│   ├── .init_db.py                     # DB初期化スクリプト
│   └── attendance.db                   # SQLiteデータベース本体
│
├── logs/                               # ログ出力ディレクトリ
│   └── attendance.log                  # 勤怠関連ログファイル
│
├── shared/                             # 共通処理・設定
│   ├── config.py                       # 環境変数や設定
│   └── error_handler.py                # 共通エラーハンドリング
│
├── web/                                # フロントエンド（React + TypeScript）
│   ├── .env                            # フロントエンド用環境変数（APIのURLなど）
│   ├── public/                         # 静的ファイル（HTML, faviconなど）
│   ├── src/                            # Reactアプリ本体
│   │   ├── api/                        # APIクライアント（fetchラッパー）
│   │   │   └── attendance.ts
│   │   ├── hooks/                      # カスタムフック
│   │   │   └── useGetAttendance.ts
│   │   ├── pages/                      # ページごとのUIコンポーネント
│   │   │   └── HomePage/
│   │   │       ├── HomePage.tsx
│   │   │       └── HomePage.css
│   │   ├── App.tsx                     # アプリのルートコンポーネント
│   │   ├── App.css
│   │   ├── index.tsx                   # アプリのエントリーポイント
│   │   └── index.css
│   ├── package.json                    # 依存パッケージとスクリプト定義
│   └── tsconfig.json                   # TypeScriptコンパイル設定
│
├── worker/                             # バックグラウンド処理（ワーカー）
│   ├── __init__.py
│   └── worker_runner.py                # タスク実行ループ
│
├── .env                                # バックエンド用環境変数
├── requirements.txt                    # Python依存パッケージ定義
├── start_api_server.sh                 # APIサーバー起動スクリプト
├── start_web_server.sh                 # Webサーバー起動スクリプト（React）
└── start_worker.sh                     # ワーカー起動スクリプト
