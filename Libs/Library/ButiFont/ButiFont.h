#ifndef BUTIFONT_H
#define BUTIFONT_H

#pragma once

#ifdef BUTIFONT_EXPORTS
#define BUTIFONT_API __declspec(dllexport)
#else
#define BUTIFONT_API __declspec(dllimport)
#endif

namespace ButiFont {

/// <summary>
/// 文字ごとの情報
/// </summary>
class FontGlyph
{
public:
	std::uint32_t    Codepoint : 31;
	std::uint32_t    Visible : 1;
	float           AdvanceX;
	/// <summary>
	/// 座標
	/// </summary>
	float           X0, Y0, X1, Y1;
	/// <summary>
	/// UV
	/// </summary>
	float           U0, V0, U1, V1;

};

/// <summary>
/// 言語の指定
/// </summary>
enum class FontLanguage {
	FONTLANG_JAPANESE, FONTLANG_ENG
};

/// <summary>
/// フォントの使用情報の取得
/// </summary>
class IFontInformation {
public:
	virtual const ButiFont::FontGlyph* * FindGlyphs_utf8(const char* arg_srcStr, std::int32_t& arg_ref_glyphSize) const= 0;
	virtual std::int32_t GetSize()const = 0;
};

/// <summary>
/// True Type Font　規格のフォントファイルの読み込み
/// </summary>
/// <param name="arg_filePath">ファイルパス</param>
/// <param name="size">フォント解像度</param>
/// <param name="arg_lang">使用する言語</param>
/// <param name="arg_output_width">フォントアトラスの横サイズ(出力)</param>
/// <param name="arg_output_height">フォントアトラスの縦サイズ(出力)</param>
/// <param name="arg_output_atlasTextureData">フォントアトラスのピクセルデータ(出力)</param>
/// <param name="arg_output_fontInfo">フォントの使用情報(出力)</param>
/// <returns></returns>
extern BUTIFONT_API void LoadTTF(const char* arg_filePath, const std::int32_t size, ButiFont::FontLanguage arg_lang,  std::int32_t* arg_output_width,  std::int32_t* arg_output_height,unsigned char** arg_output_atlasTextureData, IFontInformation** arg_output_fontInfo);

/// <summary>
/// ButiFontが確保したメモリの解放
/// </summary>
/// <param name="arg_release">解放するメモリのアドレス</param>
/// <returns></returns>
extern BUTIFONT_API void ReleaseMemory(void* arg_release);
}


#endif // !BUTIFONT_H
