#pragma once
#ifndef _SceneManager_H_
#define _SceneManager_H_

#include"stdafx.h"
#include<mutex>
namespace ButiEngine {
class IApplication;
#define FPS_DEFAULT 60
class SceneManager :public ISceneManager
{
public:
	SceneManager(Value_weak_ptr<IApplication> arg_vwp_app);

	void Update()override;
	void Initialize()override;
	void PreInitialize()override;
	void RemoveScene(const std::string& arg_sceneName)override;
	void ChangeScene(const std::string& arg_sceneName, float sceneChangeDalay = 0)override;
	void SetScene(const std::string& arg_sceneName, Value_ptr<IScene> vlp_scene)override;
	void SetScene_Init(const std::string& arg_sceneName, Value_ptr<IScene> vlp_scene)override;


	void LoadScene(const std::string& arg_sceneName, Value_ptr<SceneInformation> vlp_scene = nullptr) override;
	void LoadScene_async(const std::string& arg_sceneName) override;
	void LoadScene_Init(const std::string& arg_sceneName, Value_ptr<SceneInformation> vlp_scene = nullptr) override;
	void ReloadScene()override;
	void ReloadScene(const std::string& arg_sceneName)override;
	void RenewalScene()override;
	void RenewalSubScene();
	void ClearSubScene();
	bool IsLoading()override;
	Value_ptr<IApplication> GetApplication()const override;
	inline Value_ptr<IScene> GetCurrentScene()const override {
		return currentScene;
	}
	inline Value_ptr<IScene> GetCurrentSubScene()const {
		if (list_subScene.GetSize()) {
			return list_subScene.GetLast();
		}
		else {
			return nullptr;
		}
	}
	inline const List<Value_ptr<IScene>>& GetCurrentSubScenes()const {
		return list_subScene;
	}
	void Release()override;

	/// <summary>
	/// サブシーンの追加
	/// シーンはサブシーンを一つまで持てる
	/// </summary>
	void PushSubScene(const std::string& arg_sceneName) override;
	/// <summary>
	/// サブシーンの削除
	/// </summary>
	void PopSubScene()override;
	~SceneManager();
private:
	Value_ptr<IScene> currentScene;
	Value_ptr<IScene> newScene;
	List< Value_ptr<IScene>>list_subScene;
	List< Value_ptr<IScene>>list_newSubScene;
	std::map<std::string, Value_ptr<IScene>> map_iscene;
	Value_ptr<AbsoluteTimer> sceneChangeTimer;
	Value_weak_ptr<IApplication> vwp_app;
	bool isReload = false, isLoad = false;
	std::mutex mtx_load;
#ifdef _EDITORBUILD
public:
	bool SetActive(const bool arg_isActive)override;
	void SaveButtonPerform()override;
	void ReloadButtonPerform()override;
	void SceneChangeButtonPerform(const std::string& arg_sceneName)override;
	std::int32_t GetUpdateMilliSecond()const override;
	std::int32_t GetDrawMilliSecond()const override;
	float GetDrawMilliSecondAverage()const override;
	float GetUpdateMilliSecondAverage()const override;
private:
	void ShowGUI();
	void ReloadCheck();
	bool isActive = false;
	bool isPlaying = false;
	Value_ptr<ButiRendering::IResource_Texture>vlp_texture;
	std::string reloadSceneName;
	std::uint32_t startCount = 0;
	TextureTag screenTextureTag;

	std::timespec befUpdateTs, currentUpdateTs;
	std::timespec befDrawTs, currentDrawTs;
	std::int32_t currentDrawTime, currentUpdateTime, drawTimesBuffer[FPS_DEFAULT], updateTimesBuffer[FPS_DEFAULT];
	float drawAverageTime, updateAverageTime;
	std::int32_t timeBufIndex = 0;
	std::mutex mtx_progress,mtx_update,mtx_draw;
#endif

};



}
#endif // !_SceneManager_H_


