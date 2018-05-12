# -*- coding: utf-8 -*-

import argparse
from cui.slam_launcher import SlamLauncher


def main():
    parser = argparse.ArgumentParser(description='Little SLAM')

    parser.add_argument('--scan', '-s', action='store_true',
                        help='Scan mode')

    parser.add_argument('--odometry', '-o', action='store_true',
                        help='Odometry mode')

    parser.add_argument('--file', '-f', default='',
                        help='Data file name')

    parser.add_argument('--num', '-n', type=int, default=0,
                        help='Start scanning number')

    args = parser.parse_args()

    scan_check = args.scan          # スキャン表示のみか
    odometry_only = args.odometry   # オドメトリによる地図構築か
    filename = args.file            # データファイル名
    start_N = args.num              # 開始スキャン名

    print('scan: {}'.format(scan_check))
    print('odometry: {}'.format(odometry_only))
    print('file: {}'.format(filename))
    print('num: {}'.format(start_N))

    if filename == '':
        print('Error: no file name.')

    # ファイルを開く
    sl = SlamLauncher()
    flag = sl.set_filename(filename)
    if not flag:
        return

    sl.set_start_N(start_N)     # 開始スキャン番号の設定

    # 処理本体
    if scan_check:
        sl.show_scans()
    else:
        sl.set_odometry_only(odometry_only)
        sl.customize_framework()
        sl.run()


if __name__ == '__main__':
    main()
