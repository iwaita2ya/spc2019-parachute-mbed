#### mbed-cli セットアップ

#####
mbed-cli を入れていない場合は以下のコマンドで入れる(pipが必要)

    $ pip install mbed-cli
    $ pip install jsonschema
    $ pip install pyelftools

小ネタ: pip でローカルキャッシュを無視したい場合は`--no-cache-dir`をつける

    $ pip install --no-cache-dir <package_name>

#### mbed-os2 プロジェクトの作成

#####
`--mbedlib` オプションをつけないとOS5扱いになるので注意

※ git から clone する場合は考慮しなくてもよい…のかな

    $ mbed new <project-name> --mbedlib

#### コンパイラの設定

#####
apt で入る gcc-arm-none-eabi だと mbed compile でエラーが出るため、公式から最新のバイナリを取得してパスを通す

https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

https://os.mbed.com/docs/mbed-os/v5.11/tools/after-installation-configuring-mbed-cli.html

    例) /home/iwait/opt/gcc-arm-none-eabi/bin に最新のバイナリがある場合
    $ mbed config -G GCC_ARM_PATH "/home/iwait/opt/gcc-arm-none-eabi/bin"

#### mbed ライブラリを rev. e95d10626187 まで戻す

##### 最新の mbed ライブラリはバグがあってコンパイルエラーになるため、２つ前のバージョンに戻す

    $ cd <project-pasth>/mbed
    $ mbed update e95d10626187

#### コンパイル

    $ cd <project-pasth>
    $ mbed deploy
    $ mbed compile -m lpc1114 -t gcc_arm


