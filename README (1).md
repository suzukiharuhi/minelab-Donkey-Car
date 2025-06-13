# 【Raspberry Pi】マグネット式勤怠管理システム

## システム環境

## プロジェクト構成
```
.                                       # プロジェクトルート
├── app/                                # バックエンドアプリ（FastAPI）
│   ├── __init__.py
│   ├── main.py                         # FastAPIアプリのエントリーポイント
│   ├── api/                            # APIルーティング
│   │   ├── __init__.py
│   │   └── attendance.py               # 勤怠関連のAPIエンドポイント
│   └── schemas/                        # リクエスト/レスポンス用スキーマ
│       ├── __init__.py
│       └── attendance.py
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
```
  
## 開発言語
+ Python（バックエンド）
+ React（フロントエンド）

## I/Oエキスパンダ
Raspberry PiのGPIOを拡張する<br>
制御レジスタを通して，ピンの入出力や状態の制御を行う<br>
<img width="386" alt="Image" src="https://github.com/user-attachments/assets/9f4cce51-e79d-49f8-8618-6b43831443ef" />
### 制御レジスタ
I/Oエキスパンダ内部にある設定・制御用のメモリ領域<br>
16進数（HEX）で割り当てられ，ピンの状態の読み取りや，書き込みを行う<br>
|制御レジスタ|アドレス(HEX)|役割|
|:---:|:---:|:---|
|IODIR_A|00|ポートAのデータの方向を制御|
|IODIR_B|01|ポートBのデータの方向を制御|
|GPPU_A|0C|ポートA入力ピンのプルアップ設定|
|GPPU_B|0D|ポートB入力ピンのプルアップ設定|
|GPIO_A|12|ポートAのデータの読み取り|
|GPIO_B|13|ポートBのデータの読み取り|
|OLAT_A|14|ポートAの出力ピンの値をセット|
|OLAT_A|15|ポートBの出力ピンの値をセット|

## 磁気センサー
