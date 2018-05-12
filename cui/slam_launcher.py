# -*- coding: utf-8 -*-
import time


class SlamLauncher:
    def __init__(self):
        self.start_N        # 開始スキャン番号
        self.draw_skip      # 描画間隔
        self.odometry_only  # オドメトリによる地図構築か
        self.ipose          # オドメトリ地図構築の補助データ。初期位置の角度を0にする

        self.filename       # python版追加。データファイル名

        # self.lidar_offset = Pose2D()              # レーザスキャナとロボットの相対位置

        self.sreader = SensorDataReader()        # ファイルからのセンサデータ読み込み★
        self.pcmap = PointCloudMap()             # 点群地図★
        self.sfront = SlamFrontEnd()             # SLAMフロントエンド★
        self.mdrawer = MapDrawer()               # gnuplotによる描画★
        self.fcustom = FrameworkCustomizer()     # フレームワークの改造★

        self.start_N = 0
        self.draw_skip = 10
        self.odometry_only = False

    def set_start_N(self, N):
        self.start_N = N

    def set_odometry_only(self, p):
        self.odometry_only = p

    def run(self):
        self.mdrawer.init_gnuplot()     # gnuplot初期化
        self.mdrawer.set_aspect_ratio(-0.9)     # x軸とy軸の比（負にすると中身が一定）

        cnt = 0     # 処理の論理時刻
        if self.start_N > 0:
            self.skip_data(self.start_N)    # startNまでデータを読み飛ばす

        total_time = 0
        total_time_draw = 0
        total_time_read = 0

        scan = Scan2D()
        t0 = time.time()
        with open(self.filename, 'r') as f:
            for line in f:
                self.sreader.readline(line)     # python版追加。sreaderに1行送る
                self.sreader.load_scan(cnt, scan)  # ファイルからスキャンを1個読み込む

                if self.odometry_only:  # オドメトリによる地図構築（SLAMより優先）
                    if cnt == 0:
                        ipose = scan.pose
                        ipose.cal_rmat()

                    self.map_by_odometry(scan)
                else:
                    self.sfront.process(scan)   # SLAMによる地図構築

                t1 = time.time() - t0

                if cnt % self.draw_skip == 0:     # drawSkipおきに結果を描画
                    self.mdrawer.drawMapGp(self.pcmap)
                t2 = time.time() - t0

                ++cnt   # 論理時刻更新

                t3 = time.time() - t0
                total_time = t3  # 全体処理時間
                total_time_draw += (t2 - t1)    # 描画時間の合計
                total_time_read += (t3 - t2)    # ロード時間の合計

                print('---- SlamLauncher: cnt=%lu ends ----'.formart(cnt))

        print('Elapsed time: mapping=%g, drawing=%g, reading=%g'.format(
            total_time - total_time_draw - total_time_read, total_time_draw, total_time_read))
        print('SlamLauncher finished')

        '''
        不要
        # 処理終了後も描画画面を残すためにsleepで無限ループにする。ctrl - Cで終了。
        while True
            usleep(1000000)     # Linuxではusleep
        '''

    def show_scans(self):
        pass

    def map_by_odometry(self, scan):
        pass

    def set_filename(self, filename):
        self.filename = filename

    def skip_data(self, num):
        pass

    def customize_framework(self):
        self.fcustom.set_slam_front_end(self.sfront)
        self.fcustom.make_framework()

        self.fcustom.customize_G()  # 退化の対処をしない
        self.fcustom.customize_H()  # 退化の対処をする
        self.fcustom.customize_I()  # ループ閉じ込みをする

        self.pcmap = self.fcustom.get_point_cloud_map()     # customizeの後にやること
