#!/usr/bin/env python3

import urllib.request
import os

data_urls = [
    "https://tudatalib.ulb.tu-darmstadt.de/bitstream/handle/tudatalib/2086/2019-09-10-11-42-04.bag"
]


def download_file(url, path):
    conn = urllib.request.urlopen(url)
    with open(path, 'wb') as f:
        size = int(conn.getheader("Content-Length"))
        downloaded = 0
        block_size = 16384
        while True:
            buff = conn.read(block_size)
            if not buff:
                break
            downloaded += len(buff)
            f.write(buff)
            status = r"{:10d}MB [{:3.2f}%]".format(downloaded // 1024**2, downloaded * 100 / size)
            status = status + chr(8) * (len(status) + 1)
            print(status, end='', flush=True)
        f.close()


if __name__ == '__main__':
    if not os.path.isdir('./benchmarks/data'):
        os.mkdir('./benchmarks/data')
    for url in data_urls:
        filename = url.split('/')[-1]
        path = os.path.join('./benchmarks/data/', filename)
        if os.path.isfile(path):
            print("File '{}' already exist!".format(filename))
            while True:
                ans = input("Delete and download again? (y/n): ")
                if ans == 'n' or ans == 'N':
                    exit(0)
                if ans == 'y' or ans == 'Y':
                    break
        print("Downloading ", filename)
        download_file(url, path)
