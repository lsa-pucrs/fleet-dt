"""Screenshot helper for the VE page: python tools/shot.py URL OUT [light|dark] [step]."""
import asyncio
import sys

from playwright.async_api import async_playwright


async def main(url, out, scheme="light", step=None):
    async with async_playwright() as p:
        b = await p.chromium.launch()
        pg = await b.new_page(viewport={"width": 1440, "height": 900}, color_scheme=scheme)
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.on("console", lambda m: errs.append(m.text) if m.type == "error" else None)
        await pg.goto(url)
        await pg.wait_for_timeout(2000)
        if step is not None:
            await pg.evaluate("(k) => { setPlaying(false); state.follow = false; show(k); }", int(step))
            await pg.wait_for_timeout(400)
        await pg.screenshot(path=out)
        print("errors:", errs)
        await b.close()


if __name__ == "__main__":
    asyncio.run(main(*sys.argv[1:]))
