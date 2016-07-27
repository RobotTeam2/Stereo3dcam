/**
 * @file 筐体をコントロールするモジュール
 *
 * コントロール用コマンドは後勝ちで、前のコマンドが実行中であっても、新たに送ったコマンドを即実行する。
 */

/**
 * 筐体の移動方向を示す型
 */
typedef enum {
    R2_CHASSIS_DIR_FORWARD,     /**< 前進 */
    R2_CHASSIS_DIR_BACKWARD,    /**< 後退 */
    R2_CHASSIS_DIR_TURN_LEFT,   /**< 左回り */
    R2_CHASSIS_DIR_TURN_RIGHT,  /**< 右回り */
} r2_chassis_dir_t;

/**
 * イベントの種類を表す型
 */
typedef enum {
	R2_CHASSIS_EVENT_UNKNOWN,
	R2_CHASSIS_EVENT_ACCEPT,
	R2_CHASSIS_EVENT_COMPLETE,
	R2_CHASSIS_EVENT_ABORT,
	R2_CHASSIS_EVENT_INFORMATION
} r2_chassis_event_t;

/**
 * コマンド実行状態通知を受けるリスナーの型
 *
 * 中で長い時間をブロックしてはならない。
 * 長い処理が必要の場合はThreadを作ってください。
 *
 * @param requestID	コマンドを一意に表わす ID
 * @param eventType	イベントの種類
 * @param eventArg	イベントのデータ
 */
typedef void (*r2_chassis_listener_t)(long requestID, r2_chassis_event_t eventType, void *eventArg);

/**
 * 筐体を初期化する
 *
 * @param 筐体が接続しているシリアルポートのデバイスファイル。例： "/dev/ttyACM0"
 * @return 0 は成功、それ以外はエラーコード
 */
int r2_chassis_init(const char *device);

/**
 * 筐体の終了処理をする
 */
void r2_chassis_finalize(void);

/**
 * 筐体の電源を入れる
 */
int r2_chassis_on(void);

/**
 * 筐体の電源を切る
 */
int r2_chassis_off(void);

/**
 * イベントの通知を受けるリスナーをセット
 *
 * @param listener	リスナー。NULLの場合はリスナーを解除する。
 */
void r2_chassis_setListener(r2_chassis_listener_t listener);

/**
 * 筐体を移動する
 *
 * @param dir	移動方向
 * @param steps	移動するステップ数。
 *				１ステップの距離はチューニングで決める。
 * @return コマンドを一意に表わすID
 */
long r2_chassis_move(r2_chassis_dir_t dir, int steps);

/**
 * 移動の速さを設定する。
 *
 * @param speed 速さ。1-255 で段階設定可能。実際の速さはチューニングで決める。
 * @return コマンドを一意に表わすID
 **/
long r2_chassis_setSpeed(int speed);

/**
 * 停止する
 * @return コマンドを一意に表わすID
 */
long r2_chassis_stop(void);

/**
 * 車輪デバッグ用コマンド
 *
 * @param wheelNo	車輪を指定する場合、0: 左, 1: 右
 * @param steps		移動するステップ数。負の数の指定も可。
 * @return コマンドを一意に表わすID
 */
long r2_chassis_debug_wheel(int wheelNo, int steps);
