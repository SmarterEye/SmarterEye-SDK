#include "smarterFont.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include <string.h>
#include <float.h>

smarterFont::smarterFont()
{
}

smarterFont::~smarterFont()
{
}

const char* g_TypeFace[] = {
    "JZ",
    "MWRFRT RYQZR[SZRY",
    "JZNFNM VFVM",
    "G]OFOb UFUb JQZQ JWZW",
    "H\\PBP_ TBT_ YIWGTFPFMGKIKKLMMNOOUQWRXSYUYXWZT[P[MZKX",
    "F^[FYGVHSHPGNFLFJGIIIKKMMMOLPJPHNF [FI[ YTWTUUTWTYV[X[ZZ[X[VYT",
    "E_\\O\\N[MZMYNXPVUTXRZP[L[JZIYHWHUISJRQNRMSKSIRGPFNGMIMKNNPQUXWZY[[[\\Z\\Y",
    "NVRFRM",
    "KYVBTDRGPKOPOTPYR]T`Vb",
    "KYNBPDRGTKUPUTTYR]P`Nb",
    "JZRLRX MOWU WOMU",
    "E_RIR[ IR[R",
    "MWSZR[QZRYSZS\\R^Q_",
    "E_IR[R",
    "MWRYQZR[SZRY",
    "G][BIb",
    "H\\QFNGLJKOKRLWNZQ[S[VZXWYRYOXJVGSFQF",
    "H\\NJPISFS[",
    "H\\LKLJMHNGPFTFVGWHXJXLWNUQK[Y[",
    "H\\MFXFRNUNWOXPYSYUXXVZS[P[MZLYKW",
    "H\\UFKTZT UFU[",
    "H\\WFMFLOMNPMSMVNXPYSYUXXVZS[P[MZLYKW",
    "H\\XIWGTFRFOGMJLOLTMXOZR[S[VZXXYUYTXQVOSNRNOOMQLT",
    "H\\YFO[ KFYF",
    "H\\PFMGLILKMMONSOVPXRYTYWXYWZT[P[MZLYKWKTLRNPQOUNWMXKXIWGTFPF",
    "H\\XMWPURRSQSNRLPKMKLLINGQFRFUGWIXMXRWWUZR[P[MZLX",
    "MWRMQNROSNRM RYQZR[SZRY",
    "MWRMQNROSNRM SZR[QZRYSZS\\R^Q_",
    "F^ZIJRZ[",
    "E_IO[O IU[U",
    "F^JIZRJ[",
    "I[LKLJMHNGPFTFVGWHXJXLWNVORQRT RYQZR[SZRY",
    "DaWNVLTKQKOLNMMOMRNTOUQVTVVUWS WKWSXUYV[V\\U]S]O\\L[JYHWGTFQFNGLHJJILHOHRIUJWLYNZQ[T[WZYY",
    "I[RFJ[ RFZ[ MTWT",
    "G\\KFK[ KFTFWGXHYJYLXNWOTP KPTPWQXRYTYWXYWZT[K[",
    "H]ZKYIWGUFQFOGMILKKNKSLVMXOZQ[U[WZYXZV",
    "G\\KFK[ KFRFUGWIXKYNYSXVWXUZR[K[",
    "H[LFL[ LFYF LPTP L[Y[",
    "HZLFL[ LFYF LPTP",
    "H]ZKYIWGUFQFOGMILKKNKSLVMXOZQ[U[WZYXZVZS USZS",
    "G]KFK[ YFY[ KPYP",
    "NVRFR[",
    "JZVFVVUYTZR[P[NZMYLVLT",
    "G\\KFK[ YFKT POY[",
    "HYLFL[ L[X[",
    "F^JFJ[ JFR[ ZFR[ ZFZ[",
    "G]KFK[ KFY[ YFY[",
    "G]PFNGLIKKJNJSKVLXNZP[T[VZXXYVZSZNYKXIVGTFPF",
    "G\\KFK[ KFTFWGXHYJYMXOWPTQKQ",
    "G]PFNGLIKKJNJSKVLXNZP[T[VZXXYVZSZNYKXIVGTFPF SWY]",
    "G\\KFK[ KFTFWGXHYJYLXNWOTPKP RPY[",
    "H\\YIWGTFPFMGKIKKLMMNOOUQWRXSYUYXWZT[P[MZKX",
    "JZRFR[ KFYF",
    "G]KFKULXNZQ[S[VZXXYUYF",
    "I[JFR[ ZFR[",
    "F^HFM[ RFM[ RFW[ \\FW[",
    "H\\KFY[ YFK[",
    "I[JFRPR[ ZFRP",
    "H\\YFK[ KFYF K[Y[",
    "KYOBOb OBVB ObVb",
    "G]IL[b",
    "KYUBUb NBUB NbUb",
    "G]JTROZT JTRPZT",
    "I[J[Z[",
    "LXPFUL PFOGUL",
    "I\\XMX[ XPVNTMQMONMPLSLUMXOZQ[T[VZXX",
    "H[LFL[ LPNNPMSMUNWPXSXUWXUZS[P[NZLX",
    "I[XPVNTMQMONMPLSLUMXOZQ[T[VZXX",
    "I\\XFX[ XPVNTMQMONMPLSLUMXOZQ[T[VZXX",
    "I[LSXSXQWOVNTMQMONMPLSLUMXOZQ[T[VZXX",
    "MYWFUFSGRJR[ OMVM",
    "I\\XMX]W`VaTbQbOa XPVNTMQMONMPLSLUMXOZQ[T[VZXX",
    "I\\MFM[ MQPNRMUMWNXQX[",
    "NVQFRGSFREQF RMR[",
    "MWRFSGTFSERF SMS^RaPbNb",
    "IZMFM[ WMMW QSX[",
    "NVRFR[",
    "CaGMG[ GQJNLMOMQNRQR[ RQUNWMZM\\N]Q][",
    "I\\MMM[ MQPNRMUMWNXQX[",
    "I\\QMONMPLSLUMXOZQ[T[VZXXYUYSXPVNTMQM",
    "H[LMLb LPNNPMSMUNWPXSXUWXUZS[P[NZLX",
    "I\\XMXb XPVNTMQMONMPLSLUMXOZQ[T[VZXX",
    "KXOMO[ OSPPRNTMWM",
    "J[XPWNTMQMNNMPNRPSUTWUXWXXWZT[Q[NZMX",
    "MYRFRWSZU[W[ OMVM",
    "I\\MMMWNZP[S[UZXW XMX[",
    "JZLMR[ XMR[",
    "G]JMN[ RMN[ RMV[ ZMV[",
    "J[MMX[ XMM[",
    "JZLMR[ XMR[P_NaLbKb",
    "J[XMM[ MMXM M[X[",
    "KYTBQEPHPJQMSOSPORSTSUQWPZP\\Q_Tb",
    "NVRBRb",
    "KYPBSETHTJSMQOQPURQTQUSWTZT\\S_Pb",
    "F^IUISJPLONOPPTSVTXTZS[Q ISJQLPNPPQTTVUXUZT[Q[O"
};

bool clipLine(smarterSize img_size, smarterPoint& pt1, smarterPoint& pt2)
{
    int c1, c2;
    int right = img_size.width - 1, bottom = img_size.height - 1;

    if (img_size.width <= 0 || img_size.height <= 0)
        return false;

    int &x1 = pt1.x, &y1 = pt1.y, &x2 = pt2.x, &y2 = pt2.y;
    c1 = (x1 < 0) + (x1 > right) * 2 + (y1 < 0) * 4 + (y1 > bottom) * 8;
    c2 = (x2 < 0) + (x2 > right) * 2 + (y2 < 0) * 4 + (y2 > bottom) * 8;

    if ((c1 & c2) == 0 && (c1 | c2) != 0)
    {
        int a;
        if (c1 & 12)
        {
            a = c1 < 8 ? 0 : bottom;
            x1 += (int)((double)(a - y1) * (x2 - x1) / (y2 - y1));
            y1 = a;
            c1 = (x1 < 0) + (x1 > right) * 2;
        }
        if (c2 & 12)
        {
            a = c2 < 8 ? 0 : bottom;
            x2 += (int)((double)(a - y2) * (x2 - x1) / (y2 - y1));
            y2 = a;
            c2 = (x2 < 0) + (x2 > right) * 2;
        }
        if ((c1 & c2) == 0 && (c1 | c2) != 0)
        {
            if (c1)
            {
                a = c1 == 1 ? 0 : right;
                y1 += (int)((double)(a - x1) * (y2 - y1) / (x2 - x1));
                x1 = a;
                c1 = 0;
            }
            if (c2)
            {
                a = c2 == 1 ? 0 : right;
                y2 += (int)((double)(a - x2) * (y2 - y1) / (x2 - x1));
                x2 = a;
                c2 = 0;
            }
        }
    }

    return (c1 | c2) == 0;
}

void Line(unsigned char* img, int width, int height, smarterPoint pt1, smarterPoint pt2,
          const unsigned char color[3], int channels)
{
    int dx, dy;
    int ecount;
    int ax, ay;
    int i, j;
    int x, y;
    int x_step, y_step;
    int cb = color[0];
    int cg = color[1];
    int cr = color[2];
    unsigned char *ptr = img, *tptr;
    size_t step = width * channels;

    smarterSize sizeScaled(((int)width) << 16, ((int)height) << 16);
    if (!clipLine(sizeScaled, pt1, pt2))
        return;

    dx = pt2.x - pt1.x;
    dy = pt2.y - pt1.y;

    j = dx < 0 ? -1 : 0;
    ax = (dx ^ j) - j;
    i = dy < 0 ? -1 : 0;
    ay = (dy ^ i) - i;

    if (ax > ay)
    {
        dx = ax;
        dy = (dy ^ j) - j;
        pt1.x ^= pt2.x & j;
        pt2.x ^= pt1.x & j;
        pt1.x ^= pt2.x & j;
        pt1.y ^= pt2.y & j;
        pt2.y ^= pt1.y & j;
        pt1.y ^= pt2.y & j;

        x_step = 65536;
        y_step = (((long long) dy) << 16) / (ax | 1);
        ecount = (int)((pt2.x - pt1.x) >> 16);
    }
    else
    {
        dy = ay;
        dx = (dx ^ i) - i;
        pt1.x ^= pt2.x & i;
        pt2.x ^= pt1.x & i;
        pt1.x ^= pt2.x & i;
        pt1.y ^= pt2.y & i;
        pt2.y ^= pt1.y & i;
        pt1.y ^= pt2.y & i;

        x_step = (((long long)dx) << 16) / (ay | 1);

        y_step = 65536;
        ecount = (int)((pt2.y - pt1.y) >> 16);
    }

    pt1.x += (65536 >> 1);
    pt1.y += (65536 >> 1);

    if (channels == 3)
    {
#define  ICV_PUT_POINT(_x,_y)   \
        x = (_x); y = (_y);             \
        if( 0 <= x && x < width && \
        0 <= y && y < height ) \
        {                               \
            tptr = ptr + y*step + x*3;  \
            tptr[0] = (unsigned char)cb;        \
            tptr[1] = (unsigned char)cg;        \
            tptr[2] = (unsigned char)cr;        \
        }

        ICV_PUT_POINT((int)((pt2.x + (65536 >> 1)) >> 16),
                      (int)((pt2.y + (65536 >> 1)) >> 16));

        if (ax > ay)
        {
            pt1.x >>= 16;

            while (ecount >= 0)
            {
                ICV_PUT_POINT((int)(pt1.x), (int)(pt1.y >> 16));
                pt1.x++;
                pt1.y += y_step;
                ecount--;
            }
        }
        else
        {
            pt1.y >>= 16;

            while (ecount >= 0)
            {
                ICV_PUT_POINT((int)(pt1.x >> 16), (int)(pt1.y));
                pt1.x += x_step;
                pt1.y++;
                ecount--;
            }
        }

#undef ICV_PUT_POINT
    }
    else if (channels == 1)
    {
#define  ICV_PUT_POINT(_x,_y) \
        x = (_x); y = (_y);           \
        if( 0 <= x && x < width && \
        0 <= y && y < height ) \
        {                           \
            tptr = ptr + y*step + x;\
            tptr[0] = (unsigned char)cb;    \
        }

        ICV_PUT_POINT((int)((pt2.x + (65536 >> 1)) >> 16),
                      (int)((pt2.y + (65536 >> 1)) >> 16));

        if (ax > ay)
        {
            pt1.x >>= 16;

            while (ecount >= 0)
            {
                ICV_PUT_POINT((int)(pt1.x), (int)(pt1.y >> 16));
                pt1.x++;
                pt1.y += y_step;
                ecount--;
            }
        }
        else
        {
            pt1.y >>= 16;

            while (ecount >= 0)
            {
                ICV_PUT_POINT((int)(pt1.x >> 16), (int)(pt1.y));
                pt1.x += x_step;
                pt1.y++;
                ecount--;
            }
        }

#undef ICV_PUT_POINT
    }
}

static inline void ICV_HLINE(unsigned char* ptr, int xl, int xr, const unsigned char* color, int pix_size)
{
    unsigned char* hline_min_ptr = (unsigned char*)(ptr)+(xl)*(pix_size);
    unsigned char* hline_end_ptr = (unsigned char*)(ptr)+(xr + 1)*(pix_size);
    unsigned char* hline_ptr = hline_min_ptr;
    if (pix_size == 1)
        memset(hline_min_ptr, *color, hline_end_ptr - hline_min_ptr);
    else//if (pix_size != 1)
    {
        if (hline_min_ptr < hline_end_ptr)
        {
            memcpy(hline_ptr, color, pix_size);
            hline_ptr += pix_size;
        }//end if (hline_min_ptr < hline_end_ptr)
        size_t sizeToCopy = pix_size;
        while (hline_ptr < hline_end_ptr)
        {
            memcpy(hline_ptr, hline_min_ptr, sizeToCopy);
            hline_ptr += sizeToCopy;
            sizeToCopy = std::min(2 * sizeToCopy, static_cast<size_t>(hline_end_ptr - hline_ptr));
        }//end while(hline_ptr < hline_end_ptr)
    }//end if (pix_size != 1)
}

static void FillConvexPoly(unsigned char *img, int width, int height, const smarterPoint* v,
                           int npts, const unsigned char* color, int channels)
{
    struct
    {
        int idx, di;
        int x, dx;
        int ye;
    }
    edge[2];

    int delta = 1 << 16 >> 1;
    int i, y, imin = 0;
    int edges = npts;
    int xmin, xmax, ymin, ymax;
    unsigned char* ptr = img;
    int pix_size = channels;
    smarterPoint p0;
    int delta1, delta2;

    delta1 = delta2 = 65536 >> 1;

    p0 = v[npts - 1];

    xmin = xmax = v[0].x;
    ymin = ymax = v[0].y;

    for (i = 0; i < npts; i++)
    {
        smarterPoint p = v[i];
        if (p.y < ymin)
        {
            ymin = p.y;
            imin = i;
        }

        ymax = std::max(ymax, p.y);
        xmax = std::max(xmax, p.x);
        xmin = std::min(xmin, p.x);

        Line(img, width, height, p0, p, color, channels);
        p0 = p;
    }

    xmin = (xmin + delta) >> 16;
    xmax = (xmax + delta) >> 16;
    ymin = (ymin + delta) >> 16;
    ymax = (ymax + delta) >> 16;

    if (npts < 3 || (int)xmax < 0 || (int)ymax < 0 || (int)xmin >= width || (int)ymin >= height)
        return;

    ymax = std::min(ymax, height - 1);
    edge[0].idx = edge[1].idx = imin;

    edge[0].ye = edge[1].ye = y = (int)ymin;
    edge[0].di = 1;
    edge[1].di = npts - 1;

    edge[0].x = edge[1].x = -65536;
    edge[0].dx = edge[1].dx = 0;

    ptr += width * channels*y;

    do
    {
        if (y < (int)ymax || y == (int)ymin)
        {
            for (i = 0; i < 2; i++)
            {
                if (y >= edge[i].ye)
                {
                    int idx0 = edge[i].idx, di = edge[i].di;
                    int idx = idx0 + di;
                    if (idx >= npts) idx -= npts;
                    int ty = 0;

                    for (; edges-- > 0; )
                    {
                        ty = (int)((v[idx].y + delta) >> 16);
                        if (ty > y)
                        {
                            int xs = v[idx0].x;
                            int xe = v[idx].x;

                            edge[i].ye = ty;
                            edge[i].dx = ((xe - xs) * 2 + (ty - y)) / (2 * (ty - y));
                            edge[i].x = xs;
                            edge[i].idx = idx;
                            break;
                        }
                        idx0 = idx;
                        idx += di;
                        if (idx >= npts) idx -= npts;
                    }
                }
            }
        }
        if (edges < 0)
            break;

        if (y >= 0)
        {
            int left = 0, right = 1;
            if (edge[0].x > edge[1].x)
            {
                left = 1, right = 0;
            }

            int xx1 = (int)((edge[left].x + delta1) >> 16);
            int xx2 = (int)((edge[right].x + delta2) >> 16);

            if (xx2 >= 0 && xx1 < width)
            {
                if (xx1 < 0)
                    xx1 = 0;
                if (xx2 >= width)
                    xx2 = width - 1;
                ICV_HLINE(ptr, xx1, xx2, color, pix_size);
            }
        }

        edge[0].x += edge[0].dx;
        edge[1].x += edge[1].dx;
        ptr += width * channels;
    } while (++y <= (int)ymax);
}

void Circle(unsigned char *img, int width, int height, smarterPoint center,
            int radius, const unsigned char *color, int fill, int channels)
{
    size_t step = width * channels;
    unsigned char* ptr = img;
    int err = 0, dx = radius, dy = 0, plus = 1, minus = (radius << 1) - 1;
    int inside = center.x >= radius && center.x < width - radius &&
            center.y >= radius && center.y < height - radius;

#define ICV_PUT_POINT( ptr, x )     \
    memcpy( ptr + (x)*channels, color, channels );

    while (dx >= dy)
    {
        int mask;
        int y11 = center.y - dy, y12 = center.y + dy, y21 = center.y - dx, y22 = center.y + dx;
        int x11 = center.x - dx, x12 = center.x + dx, x21 = center.x - dy, x22 = center.x + dy;

        if (inside)
        {
            unsigned char *tptr0 = ptr + y11 * step;
            unsigned char *tptr1 = ptr + y12 * step;

            if (!fill)
            {
                ICV_PUT_POINT(tptr0, x11);
                ICV_PUT_POINT(tptr1, x11);
                ICV_PUT_POINT(tptr0, x12);
                ICV_PUT_POINT(tptr1, x12);
            }
            else
            {
                ICV_HLINE(tptr0, x11, x12, color, channels);
                ICV_HLINE(tptr1, x11, x12, color, channels);
            }

            tptr0 = ptr + y21 * step;
            tptr1 = ptr + y22 * step;

            if (!fill)
            {
                ICV_PUT_POINT(tptr0, x21);
                ICV_PUT_POINT(tptr1, x21);
                ICV_PUT_POINT(tptr0, x22);
                ICV_PUT_POINT(tptr1, x22);
            }
            else
            {
                ICV_HLINE(tptr0, x21, x22, color, channels);
                ICV_HLINE(tptr1, x21, x22, color, channels);
            }
        }
        else if (x11 < width && x12 >= 0 && y21 < height && y22 >= 0)
        {
            if (fill)
            {
                x11 = std::max(x11, 0);
                x12 = std::min(x12, width - 1);
            }

            if ((unsigned)y11 < (unsigned)height)
            {
                unsigned char *tptr = ptr + y11 * step;

                if (!fill)
                {
                    if (x11 >= 0)
                        ICV_PUT_POINT(tptr, x11);
                    if (x12 < width)
                        ICV_PUT_POINT(tptr, x12);
                }
                else
                    ICV_HLINE(tptr, x11, x12, color, channels);
            }

            if ((unsigned)y12 < (unsigned)height)
            {
                unsigned char *tptr = ptr + y12 * step;

                if (!fill)
                {
                    if (x11 >= 0)
                        ICV_PUT_POINT(tptr, x11);
                    if (x12 < width)
                        ICV_PUT_POINT(tptr, x12);
                }
                else
                    ICV_HLINE(tptr, x11, x12, color, channels);
            }

            if (x21 < width && x22 >= 0)
            {
                if (fill)
                {
                    x21 = std::max(x21, 0);
                    x22 = std::min(x22, width - 1);
                }

                if ((unsigned)y21 < (unsigned)height)
                {
                    unsigned char *tptr = ptr + y21 * step;

                    if (!fill)
                    {
                        if (x21 >= 0)
                            ICV_PUT_POINT(tptr, x21);
                        if (x22 < width)
                            ICV_PUT_POINT(tptr, x22);
                    }
                    else
                        ICV_HLINE(tptr, x21, x22, color, channels);
                }

                if ((unsigned)y22 < (unsigned)height)
                {
                    unsigned char *tptr = ptr + y22 * step;

                    if (!fill)
                    {
                        if (x21 >= 0)
                            ICV_PUT_POINT(tptr, x21);
                        if (x22 < width)
                            ICV_PUT_POINT(tptr, x22);
                    }
                    else
                        ICV_HLINE(tptr, x21, x22, color, channels);
                }
            }
        }
        dy++;
        err += plus;
        plus += 2;

        mask = (err <= 0) - 1;

        err -= minus & mask;
        dx += mask;
        minus -= mask & 2;
    }

#undef  ICV_PUT_POINT
}

void ThickLine(unsigned char *img, int width, int height, smarterPoint p0, smarterPoint p1, const unsigned char color[3],
int thickness, int flags, int channels)
{
    static const double INV_XY_ONE = 1. / 65536;

    if (thickness <= 1)
    {
        Line(img, width, height, p0, p1, color, channels);
    }
    else
    {
        smarterPoint pt[4], dp = smarterPoint(0, 0);
        double dx = (p0.x - p1.x)*INV_XY_ONE, dy = (p1.y - p0.y)*INV_XY_ONE;
        double r = dx * dx + dy * dy;
        int oddThickness = thickness & 1;
        thickness <<= 16 - 1;

        if (fabs(r) > DBL_EPSILON)
        {
            r = (thickness + oddThickness * 32768) / std::sqrt(r);
            dp.x = (int)round(dy * r);
            dp.y = (int)round(dx * r);

            pt[0].x = p0.x + dp.x;
            pt[0].y = p0.y + dp.y;
            pt[1].x = p0.x - dp.x;
            pt[1].y = p0.y - dp.y;
            pt[2].x = p1.x - dp.x;
            pt[2].y = p1.y - dp.y;
            pt[3].x = p1.x + dp.x;
            pt[3].y = p1.y + dp.y;

            FillConvexPoly(img, width, height, pt, 4, color, channels);
        }

        for (int i = 0; i < 2; i++)
        {
            if (flags & (i + 1))
            {
                smarterPoint center;
                center.x = (int)((p0.x + (65536 >> 1)) >> 16);
                center.y = (int)((p0.y + (65536 >> 1)) >> 16);
                Circle(img, width, height, center, (thickness + (65536 >> 1)) >> 16, color, 1, channels);
            }
            p0 = p1;
        }
    }
}

static void PolyLine(unsigned char *img, int width, int height, const smarterPoint* v, int count,
                     const unsigned char color[3], int thickness, int channels)
{
    if (!v || count <= 0)
        return;

    int i = 0;
    int flags = 3;
    smarterPoint p0;

    p0 = v[i];
    for (i = 1; i < count; i++)
    {
        smarterPoint p = v[i];
        ThickLine(img, width, height, p0, p, color, thickness, flags, channels);
        p0 = p;
        flags = 2;
    }
}

void smarterFont::putText(unsigned char * buffer, int width, int height,
                          const char * text, const int org[2], int channels, double fontScale,
const unsigned char color[3], int thickness)
{
    if (buffer == NULL || org == NULL || text == NULL || color == NULL
            || width < 0 || height < 0 || (channels != 1 && channels != 3))
        return;

    int hscale = (int)round(fontScale * 65536), vscale = hscale;

    int view_x = (int)org[0] << 16;
    int view_y = ((int)org[1] << 16) - 9 * vscale;

    std::vector<smarterPoint> pts;
    pts.reserve(1 << 10);

    for (int i = 0; text[i] != '\0'; i++)
    {
        int c = (unsigned char)text[i];
        smarterPoint p;

        if (c < 32 || c >= 127)
            c = '?';

        const char *ptr = g_TypeFace[c - ' '];

        p.x = (unsigned char)ptr[0] - 'R';
        p.y = (unsigned char)ptr[1] - 'R';
        int dx = p.y*hscale;
        view_x -= p.x*hscale;
        pts.resize(0);

        for (ptr += 2;; )
        {
            if (*ptr == ' ' || !*ptr)
            {
                if (pts.size() > 1)
                    PolyLine(buffer, width, height, &pts[0], (int)pts.size(), color, thickness, channels);
                if (!*ptr++)
                    break;
                pts.resize(0);
            }
            else
            {
                p.x = (unsigned char)ptr[0] - 'R';
                p.y = (unsigned char)ptr[1] - 'R';
                ptr += 2;
                pts.push_back(smarterPoint(p.x*hscale + view_x, p.y*vscale + view_y));
            }
        }
        view_x += dx;
    }
}
