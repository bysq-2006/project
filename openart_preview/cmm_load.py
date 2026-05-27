from machine import Pin
import cmm
import gc
# 读取并注册 cmm 配置。
def load():
    try:
        fd = open('/sd/cmm_cfg.csv')
        print('loading /sd/cmm_cfg.csv')
    except:
        fd = open('/flash/cmm_cfg.csv')
        print('loading /flash/cmm_cfg.csv')
    dict = {}
    for sLine in fd:
        if sLine[:2] == '//':
            continue # // 表示注释行
        # 去掉行尾的 \n 或 \r\n
        if sLine[-2] == '\r':
            sLine = sLine[:-2]
        else:
            sLine = sLine[:-1]
        lst = sLine.split(',')
        if len(lst) < 4:
            continue
        # cmm 条目格式：
        # (0)fn,(1)unit,(2)signal,(3)hint,(4)pinobj,(5)comments
        if lst[1] == '-': # 没有 "unit" 字段
            comboName = lst[0] + '.' + lst[2]
        else:
            comboName = lst[0] + '.' + lst[1] + '.' + lst[2]
        try:
            # 尝试根据 "pinobj" 字符串创建引脚对象
            pin = Pin(lst[4])
            # 创建成功后，把这个引脚对象加入映射表
            # key 是 comboname，
            # value 是 (hint, pinobj 字符串, pinobj 指针, owner 指针)
            dict[comboName] = (lst[3], lst[4], pin, None)
        except:
            # 没有对应的引脚对象，把 pinobj 指针设为 NULL（0）
            dict[comboName] = (lst[3], lst[4], None,   None)
        del(lst)
        del(comboName)
    fd.close()
    cmm.add(dict)
    return dict
if __name__ == '__main__':
    dict = load()
